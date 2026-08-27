"""洞洞板背钩转换器：主体 STL + 已排布替换钩子 STL。"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import trimesh


class ConverterModel:
    def __init__(self):
        self.body = None; self.hook = None; self.working = None
        self.hook_parts = []
        self.selected_hook_index = 0
        self.selected_hook_indices = {0}
        self.hook_groups = []
        self.mount_point = None; self.mount_normal = None
        self.history = []
        self.redo_history = []

    def snapshot(self):
        """复制完整的可编辑状态；STL 网格在切割/贴合时会原地改变。"""
        return {
            "body": self.body.copy() if self.body is not None else None,
            "hook": self.hook.copy() if self.hook is not None else None,
            "hook_parts": [part.copy() for part in self.hook_parts],
            "selected_hook_index": self.selected_hook_index,
            "selected_hook_indices": sorted(self.selected_hook_indices),
            "hook_groups": [list(group) for group in self.hook_groups],
            "working": self.working.copy() if self.working is not None else None,
            "mount_point": self.mount_point.copy() if self.mount_point is not None else None,
            "mount_normal": self.mount_normal.copy() if self.mount_normal is not None else None,
        }

    def remember(self):
        self.history.append(self.snapshot())
        if len(self.history) > 30:
            self.history.pop(0)
        self.redo_history.clear()

    def restore(self, state):
        """恢复一个完整编辑快照，供撤销和重做共用。"""
        self.body = state["body"]
        self.hook = state["hook"]
        self.hook_parts = state["hook_parts"]
        self.selected_hook_index = min(state.get("selected_hook_index", 0), max(len(self.hook_parts) - 1, 0))
        saved_selection = state.get("selected_hook_indices")
        if saved_selection is None:
            saved_selection = [self.selected_hook_index]
        self.selected_hook_indices = {i for i in saved_selection if i < len(self.hook_parts)}
        self.hook_groups = [set(i for i in group if i < len(self.hook_parts)) for group in state.get("hook_groups", [])]
        self.hook_groups = [group for group in self.hook_groups if len(group) > 1]
        self.working = state["working"]
        self.mount_point = state["mount_point"]
        self.mount_normal = state["mount_normal"]

    def undo(self):
        if not self.history:
            return False
        state = self.history.pop()
        self.redo_history.append(self.snapshot())
        if len(self.redo_history) > 30:
            self.redo_history.pop(0)
        self.restore(state)
        return True

    def redo(self):
        if not self.redo_history:
            return False
        state = self.redo_history.pop()
        self.history.append(self.snapshot())
        if len(self.history) > 30:
            self.history.pop(0)
        self.restore(state)
        return True

    @staticmethod
    def load_mesh(path):
        path = str(path)
        suffix = Path(path).suffix.lower()
        if suffix in {".step", ".stp"}:
            return ConverterModel.load_step_mesh(path)
        if suffix == ".3mf":
            try:
                obj = trimesh.load(path, file_type="3mf", force="scene")
            except ModuleNotFoundError as exc:
                if exc.name == "lxml":
                    raise ValueError("3MF 支持需要 lxml，请运行：python -m pip install lxml") from exc
                raise
        else:
            obj = trimesh.load(path, force="scene")
        if isinstance(obj, trimesh.Scene):
            meshes = [g for g in obj.geometry.values() if isinstance(g, trimesh.Trimesh)]
            if not meshes: raise ValueError("文件中没有可用三角网格")
            obj = trimesh.util.concatenate(meshes)
        if not isinstance(obj, trimesh.Trimesh) or not len(obj.faces):
            raise ValueError("不是有效的 STL 或 3MF 网格")
        obj.remove_unreferenced_vertices(); return obj

    @staticmethod
    def load_step_mesh(path, linear_deflection=0.08):
        """使用 OCP/OpenCascade 将 STEP 三角化为可编辑的 Trimesh。"""
        try:
            from OCP.STEPControl import STEPControl_Reader
            from OCP.IFSelect import IFSelect_ReturnStatus
            from OCP.BRepMesh import BRepMesh_IncrementalMesh
            from OCP.TopExp import TopExp_Explorer
            from OCP.TopAbs import TopAbs_FACE
            from OCP.BRep import BRep_Tool
            from OCP.TopoDS import TopoDS
        except ImportError as exc:
            raise ValueError("STEP 支持需要 OCP，请在项目虚拟环境中安装 cadquery-ocp") from exc
        reader = STEPControl_Reader()
        if reader.ReadFile(path) != IFSelect_ReturnStatus.IFSelect_RetDone:
            raise ValueError(f"STEP 文件读取失败：{path}")
        reader.TransferRoots(); shape = reader.OneShape()
        if shape.IsNull():
            raise ValueError("STEP 文件没有有效实体")
        BRepMesh_IncrementalMesh(shape, float(linear_deflection), False, 0.5, True)
        vertices, faces = [], []
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            triangulation = BRep_Tool.Triangulation_s(face, face.Location())
            if triangulation is not None and triangulation.NbNodes() > 0:
                base = len(vertices)
                for i in range(1, triangulation.NbNodes() + 1):
                    p = triangulation.Node(i)
                    vertices.append((p.X(), p.Y(), p.Z()))
                for i in range(1, triangulation.NbTriangles() + 1):
                    tri = triangulation.Triangle(i).Get()
                    # STEP 面方向可能反向；Trimesh 后续会统一法线。
                    faces.append((base + tri[0] - 1, base + tri[1] - 1, base + tri[2] - 1))
            explorer.Next()
        if not faces:
            raise ValueError("STEP 三角化没有得到有效面")
        mesh = trimesh.Trimesh(vertices=np.asarray(vertices, float), faces=np.asarray(faces, int), process=True)
        mesh.remove_unreferenced_vertices(); return mesh

    def set_body(self, path): self.body = self.load_mesh(path); self.working = self.body.copy(); return self.body
    def set_hook(self, path):
        self.hook = self.load_mesh(path)
        self.hook_parts = [self.hook]
        self.selected_hook_index = 0
        self.selected_hook_indices = {0}
        self.hook_groups = []
        return self.hook

    def rebuild_hook(self):
        """从独立钩子零件重建用于渲染和导出的组合网格。"""
        if not self.hook_parts:
            self.hook = None
        elif len(self.hook_parts) == 1:
            self.hook = self.hook_parts[0]
        else:
            self.hook = trimesh.util.concatenate(self.hook_parts)
        return self.hook

    def selected_hook_part(self):
        if not self.hook_parts:
            raise ValueError("请先导入替换背钩 STL")
        self.selected_hook_index = min(max(self.selected_hook_index, 0), len(self.hook_parts) - 1)
        return self.hook_parts[self.selected_hook_index]

    def select_hook(self, index, toggle=False):
        index = min(max(index, 0), len(self.hook_parts) - 1)
        if toggle:
            if index in self.selected_hook_indices and len(self.selected_hook_indices) > 1:
                self.selected_hook_indices.remove(index)
            else:
                self.selected_hook_indices.add(index)
        else:
            self.selected_hook_indices = {index}
        self.selected_hook_index = index

    def editable_hook_indices(self):
        """组合整体优先，其次是 Shift 多选，最后是当前钩子。"""
        if not self.selected_hook_indices:
            return []
        for group in self.hook_groups:
            if self.selected_hook_index in group:
                return sorted(group)
        return sorted(self.selected_hook_indices)

    def combine_selected(self):
        selected = set(self.selected_hook_indices)
        if len(selected) < 2:
            raise ValueError("请先按 Shift 选择至少两个背钩")
        self.hook_groups = [group for group in self.hook_groups if not (group & selected)]
        self.hook_groups.append(selected)

    def ungroup_selected(self):
        selected = set(self.selected_hook_indices)
        before = len(self.hook_groups)
        self.hook_groups = [group for group in self.hook_groups if not (group & selected)]
        if before == len(self.hook_groups):
            raise ValueError("当前选择不属于任何组合")

    def delete_selected_hooks(self):
        """删除当前选中钩子，并重建组合中的索引。"""
        selected = {index for index in self.selected_hook_indices if 0 <= index < len(self.hook_parts)}
        if not selected:
            raise ValueError("请先点击选择一个背钩")
        mapping, kept = {}, []
        for old_index, part in enumerate(self.hook_parts):
            if old_index not in selected:
                mapping[old_index] = len(kept)
                kept.append(part)
        groups = []
        for group in self.hook_groups:
            remapped = {mapping[index] for index in group if index in mapping}
            if len(remapped) > 1:
                groups.append(remapped)
        self.hook_parts = kept
        self.hook_groups = groups
        self.selected_hook_indices = set()
        self.selected_hook_index = 0
        self.rebuild_hook()

    def set_mount_face(self, point, normal):
        normal = np.asarray(normal, float); normal /= max(np.linalg.norm(normal), 1e-12)
        self.mount_point, self.mount_normal = np.asarray(point, float), normal

    def cut_by_plane(self, depth=0.0):
        """以选中面为分界，保留较大的主体侧，删除较小的外伸侧。

        STL 三角面的法线可能朝内或朝外，不能用它直接决定保留哪边。
        背钩转换的合理默认是保留体积/面积较大的主体，移除外侧的小结构。
        """
        if self.working is None or self.mount_point is None: raise ValueError("请先选择主体贴合面")
        # 严格落在 STL 三角形共面位置时，slice_plane 的封口可能产生
        # 重叠/零面积三角形。向主体内侧让出 0.1 mm 容差，避免破面；
        # 用户输入非零深度时仍完全按输入值执行。
        effective_depth = float(depth)
        if abs(effective_depth) < 1e-9:
            effective_depth = 0.1
        point = self.mount_point - self.mount_normal * effective_depth
        positive = self.working.slice_plane(point, self.mount_normal, cap=True)
        negative = self.working.slice_plane(point, -self.mount_normal, cap=True)
        candidates = [m for m in (positive, negative) if m is not None and len(m.faces)]
        if len(candidates) != 2: raise ValueError("切割没有得到两个有效部分，请调整选面或切割深度")
        def size(mesh):
            volume = abs(float(mesh.volume)) if mesh.is_watertight else 0.0
            return volume if volume > 1e-9 else float(mesh.area)
        sizes = [size(m) for m in candidates]
        kept_index = int(np.argmax(sizes))
        self.working = candidates[kept_index]
        # slice_plane 的封口受 STL 法线和共面三角形影响，切后统一清理，避免
        # 重叠面、退化三角形和未封口边在渲染中形成破面。
        # trimesh 5 移除了 remove_degenerate_faces/remove_duplicate_faces，
        # 改为返回布尔掩码；兼容旧版 API，避免切割在清理阶段中断。
        if hasattr(self.working, "remove_degenerate_faces"):
            self.working.remove_degenerate_faces()
        else:
            self.working.update_faces(self.working.nondegenerate_faces())
        self.working.merge_vertices()
        if hasattr(self.working, "remove_duplicate_faces"):
            self.working.remove_duplicate_faces()
        else:
            self.working.update_faces(self.working.unique_faces())
        self.working.remove_unreferenced_vertices()
        self.working.process(validate=True)
        if not self.working.is_watertight:
            self.working.fill_holes()
        return {"kept": kept_index, "sizes": sizes, "faces": [len(m.faces) for m in candidates]}

    def align_hook(self, hook_point=None, hook_normal=None, offset=0.0):
        """将钩子贴墙面贴到主体面，同时保留明确的“向上”。

        只对齐一条法线会遗留绕法线 360 度的自由度，旧实现由库任意选择该角度，
        所以钩子贴到侧面时可能横躺。约定钩子的 -Z 是贴墙面、+Y 是向上；
        目标上方取世界 +Z 投影到贴合面内，保证不同侧面贴合后仍保持正向。
        """
        if self.hook is None or self.mount_point is None: raise ValueError("请导入背钩并选择贴合面")
        b = self.hook.bounds
        if hook_point is None: hook_point = np.mean(b, axis=0); hook_point[2] = b[0, 2]
        source_normal = np.asarray(hook_normal if hook_normal is not None else (0., 0., -1.), float)
        source_normal /= max(np.linalg.norm(source_normal), 1e-12)
        source_up = np.array([0., 1., 0.])
        source_up -= source_normal * (source_up @ source_normal)
        source_up /= max(np.linalg.norm(source_up), 1e-12)
        target_normal = -self.mount_normal
        # 侧面上“上方”固定为世界 +Z 的切向投影；若贴在顶/底面，再用 +Y。
        target_up = np.array([0., 0., 1.])
        target_up -= target_normal * (target_up @ target_normal)
        if np.linalg.norm(target_up) < 1e-6:
            target_up = np.array([0., 1., 0.])
            target_up -= target_normal * (target_up @ target_normal)
        target_up /= max(np.linalg.norm(target_up), 1e-12)
        source_right = np.cross(source_up, source_normal)
        target_right = np.cross(target_up, target_normal)
        source_frame = np.column_stack((source_right, source_up, source_normal))
        target_frame = np.column_stack((target_right, target_up, target_normal))
        t = np.eye(4); t[:3, :3] = target_frame @ source_frame.T
        transformed = trimesh.transform_points([hook_point], t)[0]
        t[:3, 3] = self.mount_point - target_normal * offset - transformed
        if not self.hook_parts:
            self.hook_parts = [self.hook]
        for part in self.hook_parts:
            part.apply_transform(t)
        self.rebuild_hook()

    def transform_hook(self, translation=(0, 0, 0), rotation=(0, 0, 0), copy=False):
        """按模型坐标轴平移/旋转钩子；rotation 为 X/Y/Z 度数。"""
        if self.hook is None:
            raise ValueError("请先导入替换背钩 STL")
        
        # hook_parts 是唯一的可编辑来源；self.hook 只是组合显示/导出网格。
        if not self.hook_parts:
            self.hook_parts = [self.hook]
        indices = self.editable_hook_indices()
        if not indices:
            raise ValueError("请先点击选择一个背钩")
        sources = [self.hook_parts[index] for index in indices]
        center = trimesh.util.concatenate(sources).centroid
        # 先做旋转（围绕质心），再做平移（在世界坐标系）
        rotation_matrix = trimesh.transformations.compose_matrix(
            angles=np.radians(np.asarray(rotation, dtype=float)),
        )
        translation_matrix = trimesh.transformations.translation_matrix(
            np.asarray(translation, dtype=float)
        )
        # 旋转围绕钩子自身中心，平移在世界坐标系
        to_center = trimesh.transformations.translation_matrix(-center)
        from_center = trimesh.transformations.translation_matrix(center)
        # 组合：先移到原点 -> 旋转 -> 移回质心 -> 平移
        matrix = translation_matrix @ from_center @ rotation_matrix @ to_center
        targets = [source.copy() if copy else source for source in sources]
        for target in targets:
            target.apply_transform(matrix)
        if copy:
            start = len(self.hook_parts)
            self.hook_parts.extend(targets)
            self.selected_hook_indices = set(range(start, start + len(targets)))
            self.selected_hook_index = start
            if len(targets) > 1:
                self.hook_groups.append(set(self.selected_hook_indices))
        self.rebuild_hook()

    def apply_hook_matrix(self, matrix, copy=False):
        if self.hook is None:
            raise ValueError("请先导入替换背钩 STL")
        matrix = np.asarray(matrix, dtype=float)
        # 只接受刚体变换；避免误把显示层/缩放矩阵写回 STL。
        if not np.all(np.isfinite(matrix)):
            raise ValueError("变换矩阵包含无效数值")
        det = np.linalg.det(matrix[:3, :3])
        if abs(det - 1.0) > 1e-4 or np.max(np.abs(matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3))) > 1e-4:
            raise ValueError(f"拒绝非刚体变换：det={det:.6f}")
        
        if not self.hook_parts:
            self.hook_parts = [self.hook]
        indices = self.editable_hook_indices()
        if not indices:
            raise ValueError("请先点击选择一个背钩")
        sources = [self.hook_parts[index] for index in indices]
        before = trimesh.util.concatenate(sources).centroid.copy()
        targets = [source.copy() if copy else source for source in sources]
        for target in targets:
            target.apply_transform(matrix)
        after = trimesh.util.concatenate(targets).centroid.copy()
        if copy:
            start = len(self.hook_parts)
            self.hook_parts.extend(targets)
            self.selected_hook_indices = set(range(start, start + len(targets)))
            self.selected_hook_index = start
            if len(targets) > 1:
                self.hook_groups.append(set(self.selected_hook_indices))
        self.rebuild_hook()
        
        return before, after

    def combined(self):
        if self.working is None:
            raise ValueError("请先导入主体模型")
        # 切除旧背钩后可以直接导出纯主体，不要求必须先导入新背钩。
        if self.hook is None or not self.hook_parts:
            return self.working.copy()
        return trimesh.util.concatenate([self.working, self.hook])

    def export(self, path): self.combined().export(path)


# ---------------------------------------------------------------------------
# 纯数学的射线/轴/平面工具函数。
#
# 设计原则：变换矩阵永远由"已知的刚体参数"直接构造（一个轴 + 一个标量），
# 绝不从任何 widget 的内部矩阵（如 vtkBoxRepresentation.GetTransform()）反解。
# translation_matrix / rotation_matrix 都保证输出正交、无缩放/剪切，
# 因此这里算出来的增量矩阵天生是刚体变换，不需要事后校验。
# ---------------------------------------------------------------------------

def screen_ray(renderer, x, y):
    """把屏幕像素坐标 (x, y) 转换为世界坐标系下的一条射线 (origin, direction)。

    使用 near/far 平面反投影，对透视和平行投影相机都成立。
    """
    renderer.SetDisplayPoint(x, y, 0.0)
    renderer.DisplayToWorld()
    near = np.array(renderer.GetWorldPoint(), dtype=float)
    if abs(near[3]) > 1e-12:
        near = near / near[3]
    renderer.SetDisplayPoint(x, y, 1.0)
    renderer.DisplayToWorld()
    far = np.array(renderer.GetWorldPoint(), dtype=float)
    if abs(far[3]) > 1e-12:
        far = far / far[3]
    direction = far[:3] - near[:3]
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        return near[:3], np.array([0.0, 0.0, 1.0])
    return near[:3], direction / norm


def axis_param_from_ray(ray_origin, ray_dir, axis_point, axis_dir):
    """求屏幕射线与一条 3D 轴线的最近点，返回沿轴线方向的参数 tc。

    轴线：point = axis_point + tc * axis_dir。
    用于平移手柄：拖动时只取该参数的变化量，天生只产生沿轴分量。
    """
    O, D = np.asarray(ray_origin, float), np.asarray(ray_dir, float)
    P0, A = np.asarray(axis_point, float), np.asarray(axis_dir, float)
    D = D / max(np.linalg.norm(D), 1e-12)
    A = A / max(np.linalg.norm(A), 1e-12)
    w0 = P0 - O
    b, d, e = D @ A, D @ w0, A @ w0
    denom = 1.0 - b * b
    # 放宽阈值：只在视线与轴几乎完全重合（夹角 < ~1°）时才拒绝。
    # 原阈值 1e-9 对应 |b| > 0.99999999，实际等轴测视图下经常触发。
    # 新阈值 1e-4 对应 |b| > 0.9999，即夹角 < 0.57°，足够宽松。
    if abs(denom) < 1e-4:
        return None  # 视线几乎与轴平行，本次拖动不可靠
    tc = (b * d - e) / denom
    return float(tc)


def ray_plane_point(ray_origin, ray_dir, plane_point, plane_normal):
    """求屏幕射线与过 plane_point、法线为 plane_normal 的平面的交点。"""
    O, D = np.asarray(ray_origin, float), np.asarray(ray_dir, float)
    C, A = np.asarray(plane_point, float), np.asarray(plane_normal, float)
    A = A / max(np.linalg.norm(A), 1e-12)
    denom = D @ A
    if abs(denom) < 1e-6:
        return None  # 视线几乎与平面平行
    t = (C - O) @ A / denom
    return O + t * D


def perpendicular_component(vec, axis):
    """把 vec 投影到垂直于 axis 的平面上，并归一化。"""
    axis = np.asarray(axis, float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    v = np.asarray(vec, float) - (np.asarray(vec, float) @ axis) * axis
    n = np.linalg.norm(v)
    if n < 1e-6:
        return None
    return v / n


def signed_angle(v_from, v_to, axis):
    """v_from 绕 axis 转到 v_to 的带符号夹角（弧度）。"""
    cross = np.cross(v_from, v_to)
    return float(np.arctan2(cross @ axis, v_from @ v_to))


GIZMO_AXES = (
    ("x", np.array([1.0, 0.0, 0.0]), "#e74c3c"),
    ("y", np.array([0.0, 1.0, 0.0]), "#2ecc71"),
    ("z", np.array([0.0, 0.0, 1.0]), "#3498db"),
)


def build_gizmo_geometry(pv_module, center, size):
    """构造 6 个手柄网格：3 个平移箭头 + 3 个旋转圆环。

    返回 {(mode, axis_name): (PolyData, 颜色, 轴向量)}。
    """
    parts = {}
    for name, axis, color in GIZMO_AXES:
        # 比旋转环更长，方便从钩子外侧抓住平移轴。
        arrow = pv_module.Arrow(start=center, direction=axis, tip_length=0.25,
                                 tip_radius=0.075, shaft_radius=0.028, scale=size * 1.45)
        parts[("translate", name)] = (arrow, color, axis)
        ring = pv_module.Disc(center=center, inner=size * 0.85, outer=size * 0.95,
                               normal=axis, r_res=1, c_res=64)
        parts[("rotate", name)] = (ring, color, axis)
    return parts


def run_gui():
    print("[启动] 开始导入依赖...")
    from PySide6 import QtCore, QtGui, QtWidgets
    print("[启动] ✓ PySide6 导入成功")
    import pyvista as pv
    print(f"[启动] ✓ PyVista {pv.__version__} 导入成功")
    from pyvistaqt import QtInteractor
    print("[启动] ✓ pyvistaqt 导入成功")
    import vtk
    print(f"[启动] ✓ VTK {vtk.vtkVersion.GetVTKVersion()} 导入成功")
    print("[启动] 所有依赖导入完成\n")

    class OrientationCubeOverlay(QtWidgets.QWidget):
        """固定在渲染窗口左下角的中文导航立方体。

        VTK 自带的默认相机标记在不同平台会退化成无文字轴标记；这里用 Qt
        直接绘制目标立方体，点击面切换正交视图，拖动旋转相机，双击回到等轴测。
        """
        def __init__(self, window, parent):
            super().__init__(parent)
            self.window = window
            # 轴标签和箭头端需要留在控件范围内；原尺寸会裁掉 X/Y/Z。
            self.setFixedSize(245, 225)
            self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
            self.setMouseTracking(True)
            self._press_pos = None
            self._dragging = False
            self._faces = {}
            self._local_press = None
            self._local_dragging = False
            # 单击跳转面视图延迟 220ms 执行：若 220ms 内出现第二次点击（双击），
            # 取消跳转并回到等轴测，避免“先跳转到该面、再恢复初始视图”的闪烁。
            self._view_timer = QtCore.QTimer(self)
            self._view_timer.setSingleShot(True)
            self._view_timer.timeout.connect(self._fire_scheduled_view)
            self._scheduled_face = None
            self._double_clicked = False
            self._font = QtGui.QFont("PingFang SC", 25, QtGui.QFont.Bold)
            self._repaint_timer = QtCore.QTimer(self)
            self._repaint_timer.timeout.connect(self.update)
            self._repaint_timer.start(40)

        def _project_faces(self):
            """根据主渲染器相机，把六个世界坐标面投影到控件平面。"""
            camera = self.window.plot.renderer.GetActiveCamera()
            position = np.asarray(camera.GetPosition(), float)
            focal = np.asarray(camera.GetFocalPoint(), float)
            view_dir = focal - position
            view_dir /= max(np.linalg.norm(view_dir), 1e-12)
            up = np.asarray(camera.GetViewUp(), float)
            up -= view_dir * (up @ view_dir); up /= max(np.linalg.norm(up), 1e-12)
            right = np.cross(view_dir, up); right /= max(np.linalg.norm(right), 1e-12)
            up = np.cross(right, view_dir); up /= max(np.linalg.norm(up), 1e-12)
            # 留出蓝色 Z 轴上方的标签空间，整体比例更接近参考导航立方体。
            # 向下留白，保证 Z 标签能完整显示在蓝色箭头上方。
            center = np.array([120., 130.]); scale = 40.
            self._projection_basis = (right, up, center, scale)
            verts = np.array([(x, y, z) for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])
            def project(points):
                points = np.asarray(points, float)
                return [QtCore.QPointF(center[0] + scale * (p @ right), center[1] - scale * (p @ up)) for p in points]
            faces = {
                "right": ([(1,-1,-1),(1,1,-1),(1,1,1),(1,-1,1)], np.array([1.,0,0]), "右", "#e5e8ec"),
                "left": ([(-1,1,-1),(-1,-1,-1),(-1,-1,1),(-1,1,1)], np.array([-1.,0,0]), "左", "#e7eaee"),
                "front": ([(-1,1,-1),(1,1,-1),(1,1,1),(-1,1,1)], np.array([0.,1,0]), "前", "#ffffff"),
                "back": ([(1,-1,-1),(-1,-1,-1),(-1,-1,1),(1,-1,1)], np.array([0.,-1,0]), "后", "#eef0f3"),
                "top": ([(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)], np.array([0.,0,1.]), "顶", "#f6f6f6"),
                "bottom": ([(-1,1,-1),(1,1,-1),(1,-1,-1),(-1,-1,-1)], np.array([0.,0,-1.]), "底", "#dfe3e8"),
            }
            camera_side = position - focal
            projected = []
            for name, (points, normal, label, color) in faces.items():
                # 只绘制朝向当前相机的面，背面在旋转到背后时自然出现。
                if normal @ camera_side <= -1e-5:
                    continue
                poly = QtGui.QPolygonF(project(points))
                depth = float(np.mean([np.dot(np.asarray(p), view_dir) for p in points]))
                projected.append((depth, name, poly, label, color))
            projected.sort(key=lambda item: item[0], reverse=True)
            return projected

        def paintEvent(self, _event):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing, True)
            # 透明 QWidget 不会自动擦除上一帧。显式用 Source 清屏，否则相机
            # 每转一帧都会残留一个旧立方体，形成截图中的重影。
            p.setCompositionMode(QtGui.QPainter.CompositionMode_Source)
            p.fillRect(self.rect(), QtCore.Qt.transparent)
            p.setCompositionMode(QtGui.QPainter.CompositionMode_SourceOver)
            projected = self._project_faces()
            self._faces = {name: poly for _, name, poly, _, _ in projected}
            # XYZ 轴属于立方体的三维世界，而非 UI 的二维装饰。它们从同一个
            # 本地坐标原点 (-1,-1,-1) 出发，并且先于立方体面绘制；因此面会自然
            # 遮住背后的线段，等轴测时三条可见，其他视角则按遮挡减少可见部分。
            p.setFont(QtGui.QFont("Arial", 15, QtGui.QFont.Bold))
            axis_labels = []
            def axis(a, b, color, label):
                """绘制带箭头的三维坐标轴；标签与箭头端保持足够间距。"""
                delta = b - a
                length = max((delta.x() ** 2 + delta.y() ** 2) ** 0.5, 1.0)
                direction = QtCore.QPointF(delta.x() / length, delta.y() / length)
                normal = QtCore.QPointF(-direction.y(), direction.x())
                color = QtGui.QColor(color)
                p.setPen(QtGui.QPen(color, 2.4, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(a, b)
                # 轴尖是实际箭头，不依赖一个字母来暗示方向。
                head = QtGui.QPolygonF([
                    b,
                    b - direction * 11.0 + normal * 6.0,
                    b - direction * 11.0 - normal * 6.0,
                ])
                p.setPen(QtCore.Qt.NoPen); p.setBrush(color); p.drawPolygon(head)
                # Z 和其余轴一致，标签位于箭头继续延伸的方向上。
                label_center = b + direction * 18.0 + normal * 2.0
                axis_labels.append((label_center, label, color))
            right, up, center, scale = getattr(self, "_projection_basis", (np.array([1.,0,0]), np.array([0.,1.,0]), np.array([100.,92.]), 46.))
            def project_world(point):
                point = np.asarray(point, float)
                return QtCore.QPointF(center[0] + scale * (point @ right), center[1] - scale * (point @ up))
            origin_3d = np.array([-1.0, -1.0, -1.0])
            origin = project_world(origin_3d)
            axis(origin, project_world((1.85, -1, -1)), "#ff6b6b", "X")
            axis(origin, project_world((-1, 1.85, -1)), "#57d68d", "Y")
            axis(origin, project_world((-1, -1, 1.85)), "#57a8ff", "Z")
            # 轴之后绘制面，得到真实的前后遮挡关系。
            for _, _name, poly, label, color in projected:
                p.setBrush(QtGui.QColor(color)); p.setPen(QtGui.QPen(QtGui.QColor("#aeb5bd"), 1.2)); p.drawPolygon(poly)
            for _, name, poly, label, _color in projected:
                self._draw_face_label(p, name, poly, label)
            # 标签最后绘制，始终在箭头端外侧，不会被立方体面盖掉。
            p.setFont(QtGui.QFont("Arial", 16, QtGui.QFont.Bold))
            for center_point, label, color in axis_labels:
                p.setPen(color)
                p.drawText(QtCore.QRectF(center_point.x() - 10, center_point.y() - 12, 20, 24),
                           QtCore.Qt.AlignCenter, label)
            p.end()

        def _draw_face_label(self, painter, face_name, polygon, label):
            """将面名作为固定尺寸的贴花，透视投影到所属立方体面。"""
            if polygon.count() != 4:
                return
            # 每个面都使用同一张 100 x 100 的逻辑贴花。这个源坐标不会随着
            # 视图改变字号；QTransform 只负责将它和对应的三维面一起投影。
            source = QtGui.QPolygonF([
                QtCore.QPointF(-50, -50), QtCore.QPointF(50, -50),
                QtCore.QPointF(50, 50), QtCore.QPointF(-50, 50),
            ])
            transform = QtGui.QTransform()
            if not QtGui.QTransform.quadToQuad(source, polygon, transform):
                return
            painter.save()
            painter.setWorldTransform(transform, False)
            # 六个面有各自的本地“字朝上”方向。这里先统一文字的物理摆放，
            # 再投影到面上；默认等轴视图中可见的 顶 / 前 / 右 都直接可读，
            # 此后文字仍会随本面一起旋转，不会变成屏幕悬浮文字。
            face_orientation = {
                "top": (-1, 1),
                "front": (-1, -1),
                "right": (1, -1),
                "left": (-1, -1),
                "back": (1, -1),
                "bottom": (-1, 1),
            }
            sx, sy = face_orientation[face_name]
            painter.scale(sx, sy)
            # 文字必须完全留在本面，旋转到边缘时也不会漏到相邻面上。
            painter.setClipRect(QtCore.QRectF(-50, -50, 100, 100))
            font = QtGui.QFont("PingFang SC", 46, QtGui.QFont.DemiBold)
            painter.setFont(font)
            painter.setPen(QtGui.QColor("#101010"))
            painter.drawText(QtCore.QRectF(-45, -46, 90, 92), QtCore.Qt.AlignCenter, label)
            painter.restore()

        def _face_at(self, pos):
            self._faces = {name: poly for _, name, poly, _, _ in self._project_faces()}
            for name, poly in self._faces.items():
                if poly.containsPoint(QtCore.QPointF(pos), QtCore.Qt.OddEvenFill):
                    return name
            return None

        def local_from_display(self, display_pos):
            """VTK 左下原点显示坐标 -> 覆盖层 Qt 左上原点坐标。"""
            rw, rh = self.window.plot.renderer.GetSize()
            pw, ph = max(self.window.plot.width(), 1), max(self.window.plot.height(), 1)
            x = float(display_pos[0]) * pw / max(rw, 1) - self.x()
            y = (rh - float(display_pos[1])) * ph / max(rh, 1) - self.y()
            return QtCore.QPointF(x, y)

        def display_press(self, display_pos):
            local = self.local_from_display(display_pos)
            # 只在立方体/轴附近拦截，其他画面区域仍交给模型相机。
            if not QtCore.QRectF(14, 6, 216, 208).contains(local):
                return False
            self._local_press = local; self._local_dragging = False
            return True

        def display_move(self, display_pos):
            if self._local_press is None:
                return False
            local = self.local_from_display(display_pos)
            delta = local - self._local_press
            if delta.manhattanLength() >= 2:
                self._local_dragging = True; self._local_press = local
                camera = self.window.plot.renderer.GetActiveCamera()
                camera.Azimuth(float(delta.x()) * 0.65)
                camera.Elevation(float(-delta.y()) * 0.65)
                camera.OrthogonalizeViewUp(); self.window.plot.render()
            return True

        def display_release(self, display_pos):
            if self._local_press is None:
                return False
            local = self.local_from_display(display_pos)
            if not self._local_dragging:
                if self._double_clicked:
                    # 双击序列里的第二次松开：不再触发单击跳转。
                    self._double_clicked = False
                else:
                    face = self._face_at(local)
                    if face:
                        self._schedule_face_view(face)
            self._local_press = None; self._local_dragging = False
            return True

        def display_double_click(self, _display_pos):
            self._local_press = None; self._local_dragging = False
            self._double_clicked = True
            self._cancel_scheduled_view()
            self.window.set_view(self.window.plot.view_isometric)
            return True

        def _schedule_face_view(self, face):
            """延迟执行单击跳转，给双击留出判定窗口。"""
            self._scheduled_face = face
            self._view_timer.start(220)

        def _cancel_scheduled_view(self):
            self._view_timer.stop()
            self._scheduled_face = None

        def _fire_scheduled_view(self):
            if self._scheduled_face is not None:
                self.window.set_named_view(self._scheduled_face)
                self._scheduled_face = None

        def mousePressEvent(self, event):
            self._press_pos = event.position().toPoint(); self._dragging = False
            event.accept()

        def mouseMoveEvent(self, event):
            if self._press_pos is None or not (event.buttons() & QtCore.Qt.LeftButton):
                return
            now = event.position().toPoint(); delta = now - self._press_pos
            if delta.manhattanLength() < 2:
                return
            self._dragging = True; self._press_pos = now
            camera = self.window.plot.renderer.GetActiveCamera()
            camera.Azimuth(float(delta.x()) * 0.65)
            camera.Elevation(float(-delta.y()) * 0.65)
            camera.OrthogonalizeViewUp(); self.window.plot.render()
            event.accept()

        def mouseReleaseEvent(self, event):
            if not self._dragging and self._press_pos is not None:
                if self._double_clicked:
                    self._double_clicked = False
                else:
                    face = self._face_at(self._press_pos)
                    if face:
                        self._schedule_face_view(face)
            self._press_pos = None; self._dragging = False; event.accept()

        def mouseDoubleClickEvent(self, event):
            self._double_clicked = True
            self._cancel_scheduled_view()
            self.window.set_view(self.window.plot.view_isometric)
            event.accept()

    class GizmoInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
        """在默认相机操纵器上叠加手柄拖拽处理。

        命中手柄时自己处理平移/旋转，不调用父类方法，相机自然就不会跟着转，
        不需要用 AbortFlag 之类的技巧去抢事件。
        """

        def __init__(self, window):
            super().__init__()
            self.window = window
            self._camera_button_down = False
            self.AddObserver("LeftButtonPressEvent", self._on_left_down)
            self.AddObserver("MouseMoveEvent", self._on_move)
            self.AddObserver("LeftButtonReleaseEvent", self._on_left_up)
            self.AddObserver("LeftButtonDoubleClickEvent", self._on_double_click)
            self.AddObserver("KeyPressEvent", self._on_key)

        def _on_left_down(self, _obj, _event):
            pos = self.GetInteractor().GetEventPosition()
            self._camera_button_down = False
            if self.window.orientation_overlay and self.window.orientation_overlay.display_press(pos):
                return
            # 选贴合面是独占模式：此时不允许钩子或操纵器抢走点击。
            if self.window.pick_enabled:
                self.window.pick_mount_face_at(pos)
                return
            if self.window.gizmo_visible:
                if self.window.gizmo_try_select_hook(pos):
                    return
                if self.window.gizmo_try_start_drag(pos):
                    return
                # 点空白处退出钩子编辑，再将本次拖动交回正常相机。
                self.window.hide_gizmo()
            if self.window.hook_select_enabled and self.window.select_hook_at(pos, show_gizmo=True):
                return
            self._camera_button_down = True
            self.OnLeftButtonDown()

        def _on_move(self, _obj, _event):
            pos = self.GetInteractor().GetEventPosition()
            if self.window.orientation_overlay and self.window.orientation_overlay._local_press is not None:
                self.window.orientation_overlay.display_move(pos)
                return
            if self.window.gizmo_dragging:
                self.window.gizmo_update_drag(pos)
                return
            self.window.gizmo_update_hover(pos)
            self.OnMouseMove()

        def _on_left_up(self, _obj, _event):
            pos = self.GetInteractor().GetEventPosition()
            if self.window.orientation_overlay and self.window.orientation_overlay._local_press is not None:
                self.window.orientation_overlay.display_release(pos)
                return
            if self.window.gizmo_dragging:
                self.window.gizmo_finish_drag()
                return
            if self._camera_button_down:
                self._camera_button_down = False
                self.OnLeftButtonUp()

        def _on_key(self, _obj, _event):
            key = self.GetInteractor().GetKeySym().lower()
            if key == "escape":
                self.window.clear_hook_selection()
                return
            if key in ("delete", "backspace"):
                self.window.delete_selected_hooks()
                return
            if key == "z" and self.GetInteractor().GetControlKey():
                if self.GetInteractor().GetShiftKey():
                    self.window.redo()
                else:
                    self.window.undo()
                return
            if self.window.gizmo_dragging and self.GetInteractor().GetKeySym() == "c":
                drag = self.window.gizmo_drag
                drag["copy"] = not drag["copy"]
                self.window.log("拖动中切换拷贝开关：" + ("开" if drag["copy"] else "关"))

        def _on_double_click(self, _obj, _event):
            pos = self.GetInteractor().GetEventPosition()
            if self.window.orientation_overlay and self.window.orientation_overlay.display_double_click(pos):
                return

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            print("[窗口] 初始化主窗口...")
            super().__init__(); self.setWindowTitle("PegHook Studio - 洞洞板背钩转换器"); self.resize(1200, 800)
            QtWidgets.QApplication.instance().installEventFilter(self)
            print("[窗口] ✓ 主窗口创建成功")
            self.m = ConverterModel(); self.pick_enabled = False; self.hook_select_enabled = False; self.hook_actor = None
            self.body_actor = None
            self.hook_actors = {}  # vtk actor -> hook_parts index; each copied hook remains selectable.
            self.hook_actor_keys = {}  # VTK actor address -> hook_parts index, survives wrapper recreation.
            self.body_actor_key = None
            print("[窗口] ✓ 模型初始化成功")
            # --- 新版三轴箭头/圆环操纵器状态 ---
            self.gizmo_style = None; self.default_style = None
            self.gizmo_picker = None
            self.hook_picker = None
            self.gizmo_actor_info = {}       # actor -> (mode, axis_name, axis_vector)
            self.gizmo_base_colors = {}      # actor -> 原始颜色
            self.gizmo_visible = False
            self.gizmo_dragging = False
            self.gizmo_drag = None
            self.gizmo_center = None
            self.gizmo_size = 5.0
            self.gizmo_hovered = None
            self.gizmo_active = None  # (mode, axis_name, axis) 当前活跃轴，供悬浮框常驻与“应用”使用
            self.gizmo_editor = None; self.gizmo_axis_badge = None
            self.gizmo_offset_label = None
            self.gizmo_offset_origin = None  # 贴合基准（偏移读数原点）
            self.align_button = None
            self.aligned = False  # 当前是否已贴合（按钮显示“取消贴合”）
            self._pre_align_state = None
            self.ruler_active = False  # 选好贴合面后自动在查看器底部显示切割面长宽
            self.ruler_actor = None
            self.gizmo_value = None; self.gizmo_copy_box = None; self.gizmo_apply_button = None
            self.gizmo_commit_lock = False
            self.camera_orientation_widget = None
            self.orientation_marker = None
            self.orientation_overlay = None
            self.operation_list = None
            self.preset_paths = {
                "1.2mm": Path(__file__).with_name("preset_hooks") / "peg_hook_1.2mm.step",
                "5mm": Path(__file__).with_name("preset_hooks") / "peg_hook_5mm.step",
            }
            print("[窗口] 开始构建 UI...")
            self._ui()
            print("[窗口] ✓ UI 构建完成")

        def _ui(self):
            print("[UI] 创建根布局...")
            # 统一界面样式：圆角按钮/输入框、hover 反馈、配色一致。
            self.setStyleSheet("""
                QMainWindow { background: #eef1f4; }
                QWidget#workspaceRoot { background: #eef1f4; }
                QScrollArea#sidePanel { background: #f7f8fa; border-left: 1px solid #d8dde4; }
                QWidget#sidePanelContent { background: #f7f8fa; }
                QGroupBox {
                    font-weight: 700; color: #273444;
                    border: 1px solid #d9dee5; border-radius: 7px;
                    margin-top: 13px; padding: 12px 10px 10px 10px;
                    background: #ffffff;
                }
                QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #4a5868; }
                QPushButton {
                    min-height: 30px; background: #f8fafb; border: 1px solid #cfd6df; border-radius: 5px;
                    padding: 5px 10px; color: #273444; font-size: 13px; font-weight: 600;
                }
                QPushButton:hover { background: #edf2f6; border-color: #9eabb9; }
                QPushButton:pressed { background: #dce4eb; }
                QPushButton:checked { background: #2d9d5a; color: white; border-color: #25874c; }
                QPushButton:disabled { background: #f5f6f8; color: #a8b0b9; border-color: #e0e4e9; }
                QPushButton#primaryButton { background: #2c78c5; border-color: #2265aa; color: white; }
                QPushButton#primaryButton:hover { background: #226db6; }
                QPushButton#cutButton { color: #9f3e32; border-color: #e2bbb5; background: #fff8f7; }
                QPushButton#cutButton:hover { background: #fceae7; border-color: #d59288; }
                QPushButton#exportButton { background: #273444; border-color: #273444; color: white; }
                QPushButton#exportButton:hover { background: #1d2733; }
                QPushButton#historyButton { background: #ffffff; border-color: #c4ced9; color: #3c5b78; text-align: left; padding-left: 12px; }
                QPushButton#historyButton:hover { background: #eaf2fa; border-color: #7fa5c9; }
                QPushButton#floatingHistoryButton, QPushButton#floatingExportButton {
                    min-width: 42px; max-width: 42px; min-height: 38px; max-height: 38px;
                    padding: 2px; border-radius: 8px; background: rgba(255,255,255,235);
                    border: 1px solid #c4ced9; color: #3c5b78; font-weight: 700;
                }
                QPushButton#floatingHistoryButton { font-size: 21px; }
                QPushButton#floatingHistoryButton:hover, QPushButton#floatingExportButton:hover { background: #eaf2fa; border-color: #7fa5c9; }
                QPushButton#floatingExportButton { background: #273444; border-color: #273444; color: white; font-size: 12px; }
                QPushButton#floatingExportButton:hover { background: #1d2733; }
                QPushButton#compactButton { min-height: 25px; padding: 3px 7px; font-size: 12px; }
                QDoubleSpinBox, QComboBox {
                    min-height: 28px; background: #fbfcfd; border: 1px solid #cfd6df; border-radius: 5px;
                    padding: 2px 7px; color: #273444;
                }
                QDoubleSpinBox:hover, QComboBox:hover { border-color: #aeb9c5; }
                QDoubleSpinBox:focus, QComboBox:focus { border: 2px solid #3a81c9; padding: 1px 6px; }
                QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                    width: 18px; border-left: 1px solid #d7dde4; background: #f0f3f6;
                }
                QDoubleSpinBox::up-button { border-bottom: 1px solid #d7dde4; }
                QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background: #e1e7ed; }
                QDoubleSpinBox::up-button:pressed, QDoubleSpinBox::down-button:pressed { background: #ced8e2; }
                QLabel { color: #445261; }
                QLabel#statusLabel { background: #eaf1f8; border: 1px solid #d4e0ed; border-radius: 5px; color: #36516b; }
                QFrame#historyPopup { background: #ffffff; border: 1px solid #cbd5df; border-radius: 7px; }
                QListWidget { border: 1px solid #d9dee5; border-radius: 5px; background: #ffffff; padding: 3px; }
                QListWidget::item { padding: 4px 5px; border-bottom: 1px solid #eef1f4; }
                QListWidget::item:selected { background: #e5f0fb; color: #1f568d; }
                QScrollArea { border: none; background: transparent; }
                QScrollBar:vertical { width: 8px; background: transparent; margin: 5px 2px; }
                QScrollBar::handle:vertical { min-height: 30px; border-radius: 4px; background: #c7cfd9; }
                QScrollBar::handle:vertical:hover { background: #aeb9c5; }
            """)
            root = QtWidgets.QWidget(); root.setObjectName("workspaceRoot"); self.setCentralWidget(root)
            row = QtWidgets.QHBoxLayout(root); row.setContentsMargins(10, 10, 10, 10); row.setSpacing(10)
            print("[UI] 创建 PyVista 渲染器（可能需要几秒）...")
            self.plot = QtInteractor(root); self.plot.set_background("#edf0f2"); self.plot.enable_anti_aliasing("fxaa")
            print("[UI] ✓ PyVista 渲染器创建成功")
            self.plot.enable_eye_dome_lighting(); row.addWidget(self.plot, 1)
            # 编辑区右上角的悬浮工具：操作历史和快速导出，不占用右侧参数面板空间。
            self.history_button = QtWidgets.QPushButton("◷", self.plot)
            self.history_button.setObjectName("floatingHistoryButton")
            self.history_button.setToolTip("操作历史")
            self.history_button.setCursor(QtCore.Qt.PointingHandCursor)
            self.history_button.clicked.connect(self.toggle_history_popup)
            self.floating_export_button = QtWidgets.QPushButton("导出", self.plot)
            self.floating_export_button.setObjectName("floatingExportButton")
            self.floating_export_button.setToolTip("导出模型")
            self.floating_export_button.setCursor(QtCore.Qt.PointingHandCursor)
            self.floating_export_button.clicked.connect(self.save)
            self.history_button.raise_(); self.floating_export_button.raise_()
            # 侧栏放入滚动区：小屏幕纵向空间不足时滚动而非挤压按钮/文字。
            panel_host = QtWidgets.QWidget(); panel_host.setObjectName("sidePanelContent")
            panel_host.setMinimumWidth(310); panel_host.setMaximumWidth(390)
            panel = QtWidgets.QVBoxLayout(panel_host); panel.setContentsMargins(12, 12, 12, 14); panel.setSpacing(10)
            panel_scroll = QtWidgets.QScrollArea(); panel_scroll.setObjectName("sidePanel"); panel_scroll.setWidgetResizable(True)
            panel_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            panel_scroll.setWidget(panel_host)
            panel_scroll.setMinimumWidth(310); panel_scroll.setMaximumWidth(390)
            row.addWidget(panel_scroll)
            self.install_camera_orientation_widget()
            # 记录默认相机操纵器，供隐藏 gizmo 时恢复；gizmo 专用 style 只在显示操纵器时挂上去。
            self.default_style = self.plot.iren.interactor.GetInteractorStyle()
            self.gizmo_style = GizmoInteractorStyle(self)
            self.plot.iren.interactor.SetInteractorStyle(self.gizmo_style)
            self.gizmo_picker = vtk.vtkCellPicker(); self.gizmo_picker.SetTolerance(0.005)
            self.hook_picker = vtk.vtkCellPicker(); self.hook_picker.SetTolerance(0.005)
            self.body_picker = vtk.vtkCellPicker(); self.body_picker.SetTolerance(0.005)
            body_box = QtWidgets.QGroupBox("主体处理")
            body_layout = QtWidgets.QGridLayout(body_box)
            body_layout.setHorizontalSpacing(8); body_layout.setVerticalSpacing(8)
            body_import = QtWidgets.QPushButton("导入主体模型"); body_import.setObjectName("primaryButton"); body_import.clicked.connect(self.load_body)
            face_pick = QtWidgets.QPushButton("选择贴合面"); face_pick.clicked.connect(self.pick_mode)
            self.depth = self.setup_spinbox(QtWidgets.QDoubleSpinBox()); self.depth.setRange(-1000, 1000); self.depth.setDecimals(3); self.depth.setSuffix(" mm")
            cut_button = QtWidgets.QPushButton("切除外侧背钩"); cut_button.setObjectName("cutButton"); cut_button.clicked.connect(self.cut)
            body_layout.addWidget(body_import, 0, 0, 1, 2); body_layout.addWidget(face_pick, 1, 0, 1, 2)
            body_layout.addWidget(QtWidgets.QLabel("切割深度"), 2, 0); body_layout.addWidget(self.depth, 2, 1)
            body_layout.addWidget(cut_button, 3, 0, 1, 2)
            panel.addWidget(body_box)

            hook_box = QtWidgets.QGroupBox("替换背钩")
            hook_layout = QtWidgets.QGridLayout(hook_box)
            hook_layout.setHorizontalSpacing(8); hook_layout.setVerticalSpacing(8)
            self.hook_preset = QtWidgets.QComboBox(); self.hook_preset.addItem("选择内置背钩", None)
            self.hook_preset.addItem("1.2 mm", "1.2mm"); self.hook_preset.addItem("5 mm", "5mm")
            self.hook_preset.activated.connect(self.load_hook_preset)
            manual_hook = QtWidgets.QPushButton("手动导入背钩"); manual_hook.clicked.connect(self.load_hook)
            self.align_button = QtWidgets.QPushButton("背钩贴合"); self.align_button.clicked.connect(self.align)
            self.align_button.setStyleSheet("background:#2d9d5a; color:white; font-weight:bold; padding:7px;")
            hook_layout.addWidget(self.hook_preset, 0, 0); hook_layout.addWidget(manual_hook, 0, 1); hook_layout.addWidget(self.align_button, 1, 0, 1, 2)
            combine_button = QtWidgets.QPushButton("组合背钩"); combine_button.setObjectName("compactButton")
            combine_button.clicked.connect(self.combine_selected_hooks)
            ungroup_button = QtWidgets.QPushButton("解散组合"); ungroup_button.setObjectName("compactButton")
            ungroup_button.clicked.connect(self.ungroup_selected_hooks)
            group_row = QtWidgets.QHBoxLayout(); group_row.setSpacing(6)
            group_row.addWidget(combine_button); group_row.addWidget(ungroup_button)
            hook_layout.addLayout(group_row, 2, 0, 1, 2)
            panel.addWidget(hook_box)
            transform_box = QtWidgets.QGroupBox("背钩变换")
            transform_grid = QtWidgets.QGridLayout(transform_box)
            transform_grid.setHorizontalSpacing(6); transform_grid.setVerticalSpacing(5)
            translate_title = QtWidgets.QLabel("平移（mm）"); translate_title.setAlignment(QtCore.Qt.AlignCenter)
            rotate_title = QtWidgets.QLabel("旋转（度）"); rotate_title.setAlignment(QtCore.Qt.AlignCenter)
            transform_grid.addWidget(translate_title, 0, 1); transform_grid.addWidget(rotate_title, 0, 2)
            self.transform_fields = {}
            self._side_transform_timer = QtCore.QTimer(self); self._side_transform_timer.setSingleShot(True)
            self._side_transform_timer.timeout.connect(self.preview_side_transform)
            self._side_transform_finish_timer = QtCore.QTimer(self); self._side_transform_finish_timer.setSingleShot(True)
            self._side_transform_finish_timer.timeout.connect(self.finish_side_transform)
            self._side_transform_session = None
            self._side_transform_updating = False
            axis_colors = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}
            for row_index, (label, translate_key, rotate_key) in enumerate((("X", "tx", "rx"), ("Y", "ty", "ry"), ("Z", "tz", "rz")), start=1):
                swatch = QtWidgets.QLabel(label); swatch.setAlignment(QtCore.Qt.AlignCenter); swatch.setFixedSize(22, 22)
                swatch.setStyleSheet(f"background:{axis_colors[label.lower()]}; color:white; font-weight:bold; border-radius:3px;")
                transform_grid.addWidget(swatch, row_index, 0)
                for column, key in ((1, translate_key), (2, rotate_key)):
                    field = self.setup_spinbox(QtWidgets.QDoubleSpinBox()); field.setRange(-100000, 100000); field.setDecimals(2)
                    field.setFixedWidth(96)
                    field.valueChanged.connect(self.schedule_side_transform_preview)
                    field.editingFinished.connect(self.finish_side_transform)
                    self.transform_fields[key] = field; transform_grid.addWidget(field, row_index, column)
            panel.addWidget(transform_box)
            export_button = QtWidgets.QPushButton("导出模型"); export_button.setObjectName("exportButton"); export_button.clicked.connect(self.save)
            # 状态提示保留给内部逻辑使用，但不再占用侧栏布局空间。
            self.info = QtWidgets.QLabel("先导入模型"); self.info.setObjectName("statusLabel"); self.info.setWordWrap(True); self.info.setContentsMargins(8, 7, 8, 7); self.info.hide()
            self.history_popup = QtWidgets.QFrame(self, QtCore.Qt.Popup | QtCore.Qt.FramelessWindowHint)
            self.history_popup.setObjectName("historyPopup")
            history_layout = QtWidgets.QVBoxLayout(self.history_popup); history_layout.setContentsMargins(10, 9, 10, 10); history_layout.setSpacing(6)
            history_title = QtWidgets.QLabel("操作历史"); history_title.setStyleSheet("font-weight:700; color:#34495e;")
            history_layout.addWidget(history_title)
            self.operation_list = QtWidgets.QListWidget(); self.operation_list.setMinimumWidth(270); self.operation_list.setFixedHeight(255)
            self.operation_list.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
            history_layout.addWidget(self.operation_list)
            self.history_popup.hide()
            self.undo_shortcuts = []
            # 显式注册 macOS 快捷键，避免 QKeySequence.Undo 在 VTK 原生窗口
            # 或不同 Qt 平台映射下无法识别 Cmd+Z。
            undo_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Meta+Z"), self)
            undo_shortcut.setContext(QtCore.Qt.ApplicationShortcut)
            undo_shortcut.activated.connect(self.undo)
            self.undo_shortcuts.append(undo_shortcut)
            redo_shortcut = QtGui.QShortcut(QtGui.QKeySequence("Meta+Shift+Z"), self)
            redo_shortcut.setContext(QtCore.Qt.ApplicationShortcut)
            redo_shortcut.activated.connect(self.redo)
            self.undo_shortcuts.append(redo_shortcut)
            self.gizmo_editor = QtWidgets.QFrame(self.plot)
            self.gizmo_editor.setObjectName("gizmoEditor")
            self.gizmo_editor.setStyleSheet(
                "QFrame#gizmoEditor { background:#ffffff; border:1px solid #aeb7c0; border-radius:6px; }"
                "QDoubleSpinBox { min-width:98px; padding:3px 5px; }"
                "QPushButton { min-width:54px; padding:4px 10px; }"
                "QCheckBox { padding:2px 3px; }"
                "QLabel { font-family:monospace; font-size:11px; color:#444; }")
            editor_outer = QtWidgets.QVBoxLayout(self.gizmo_editor)
            editor_outer.setContentsMargins(6, 5, 6, 5); editor_outer.setSpacing(3)
            editor_row = QtWidgets.QHBoxLayout(); editor_row.setSpacing(6)
            self.gizmo_axis_badge = QtWidgets.QLabel("X")
            self.gizmo_axis_badge.setAlignment(QtCore.Qt.AlignCenter); self.gizmo_axis_badge.setFixedSize(25, 25)
            self.gizmo_value = self.setup_spinbox(QtWidgets.QDoubleSpinBox()); self.gizmo_value.setRange(-100000, 100000); self.gizmo_value.setDecimals(3)
            self.gizmo_copy_box = QtWidgets.QCheckBox("拷贝")
            self.gizmo_apply_button = QtWidgets.QPushButton("应用")
            self.gizmo_apply_button.clicked.connect(self.apply_gizmo_value)
            self.gizmo_value.lineEdit().returnPressed.connect(self.apply_gizmo_value)
            for widget in (self.gizmo_axis_badge, self.gizmo_value, self.gizmo_copy_box, self.gizmo_apply_button):
                editor_row.addWidget(widget)
            editor_outer.addLayout(editor_row)
            # 偏移读数：贴合后为 0，移动后实时显示累计位移，方便微调。
            self.gizmo_offset_label = QtWidgets.QLabel("")
            editor_outer.addWidget(self.gizmo_offset_label)
            self.gizmo_editor.hide()
            self.log("程序启动，等待导入 STL")
            panel.addStretch(1)
            panel.addWidget(export_button)
            self.enable_picking(False)

        def eventFilter(self, _watched, event):
            """VTK 获得焦点时仍可靠处理 macOS Cmd+Z / Cmd+Shift+Z。"""
            if event.type() == QtCore.QEvent.Wheel and (
                isinstance(_watched, QtWidgets.QAbstractSpinBox)
                or (isinstance(_watched, QtWidgets.QLineEdit)
                    and isinstance(_watched.parent(), QtWidgets.QAbstractSpinBox))
            ):
                # 悬停滚轮不应悄悄修改平移/旋转/切割数值；上下箭头和键盘 ↑↓ 仍可用。
                event.accept()
                return True
            if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Escape:
                self.clear_hook_selection()
                event.accept()
                return True
            if event.type() == QtCore.QEvent.KeyPress and event.key() in (QtCore.Qt.Key_Delete, QtCore.Qt.Key_Backspace):
                # 焦点在文本/数字输入框时退格是改内容，不能误删钩子。
                focus = QtWidgets.QApplication.focusWidget()
                if focus is not None and isinstance(focus, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox)):
                    return super().eventFilter(_watched, event)
                self.delete_selected_hooks()
                event.accept()
                return True
            if event.type() == QtCore.QEvent.KeyPress and event.key() == QtCore.Qt.Key_Z:
                modifiers = event.modifiers()
                if modifiers & QtCore.Qt.MetaModifier:
                    if modifiers & QtCore.Qt.ShiftModifier:
                        self.redo()
                    else:
                        self.undo()
                    event.accept()
                    return True
            return super().eventFilter(_watched, event)

        def resizeEvent(self, event):
            super().resizeEvent(event)
            if self.orientation_overlay is not None:
                self.orientation_overlay.move(12, self.plot.height() - self.orientation_overlay.height() - 12)
            if getattr(self, "history_button", None) is not None and getattr(self, "floating_export_button", None) is not None:
                margin = 12
                right = self.plot.width() - margin - self.history_button.width()
                self.history_button.move(max(margin, right), margin)
                self.floating_export_button.move(max(margin, right), margin + self.history_button.height() + 7)
                self.history_button.raise_(); self.floating_export_button.raise_()

        def install_camera_orientation_widget(self):
            self.orientation_overlay = OrientationCubeOverlay(self, self.plot)
            self.orientation_overlay.move(12, self.plot.height() - self.orientation_overlay.height() - 12)
            self.orientation_overlay.raise_()
            self.orientation_overlay.show()

        def log(self, message):
            print("[洞洞板]", message, flush=True)

        def record_operation(self, message):
            self.log(message)
            self.operation_list.insertItem(0, message)
            while self.operation_list.count() > 100:
                self.operation_list.takeItem(self.operation_list.count() - 1)

        def setup_spinbox(self, field):
            """数值框保留键盘/箭头步进，滚轮由 eventFilter 统一拦截。"""
            field.setKeyboardTracking(True)
            field.setFocusPolicy(QtCore.Qt.StrongFocus)
            field.setAccelerated(True)
            field.setSingleStep(0.5)
            field.installEventFilter(self)
            # macOS/Qt 有时把滚轮事件交给内部 QLineEdit，而不是
            # QAbstractSpinBox 本身；两层都拦截才能避免悬停时误改数值。
            field.lineEdit().installEventFilter(self)
            return field

        def toggle_history_popup(self):
            if self.history_popup.isVisible():
                self.history_popup.hide()
                return
            self.history_popup.adjustSize()
            button_pos = self.history_button.mapToGlobal(QtCore.QPoint(0, self.history_button.height() + 4))
            # 从悬浮图标左侧展开，避免遮挡右侧的背钩编辑器。
            self.history_popup.move(button_pos.x() + self.history_button.width() - self.history_popup.width(), button_pos.y())
            self.history_popup.show()
            self.history_popup.raise_()

        def reset_align_state(self):
            """贴合状态复位（导入新钩子等操作后调用）。"""
            self.aligned = False
            self.gizmo_offset_origin = None
            self._pre_align_state = None
            if self.align_button is not None:
                self.align_button.setText("背钩贴合")
                self.align_button.setStyleSheet("background:#2d9d5a; color:white; font-weight:bold; padding:7px;")
            self.update_offset_readout()

        def update_offset_readout(self):
            """刷新悬浮框里的偏移读数：贴合基准为 0，之后显示累计位移。"""
            if self.gizmo_offset_label is None:
                return
            if self.gizmo_offset_origin is None or not self.m.hook_parts:
                self.gizmo_offset_label.setText("")
                return
            indices = self.m.editable_hook_indices()
            if not indices:
                self.gizmo_offset_label.setText("")
                return
            center = trimesh.util.concatenate([self.m.hook_parts[i] for i in indices]).centroid
            delta = center - self.gizmo_offset_origin
            self.gizmo_offset_label.setText(
                f"ΔX {delta[0]:+.3f}  ΔY {delta[1]:+.3f}  ΔZ {delta[2]:+.3f}")

        def follow_gizmo_editor(self):
            """拖动中让悬浮框实时跟随移动后的钩子。"""
            if self.gizmo_active is None or not self.m.hook_parts:
                return
            _mode, _name, axis = self.gizmo_active
            indices = self.m.editable_hook_indices()
            if not indices:
                return
            center = trimesh.util.concatenate([self.m.hook_parts[i] for i in indices]).centroid
            anchor = center + axis * self.gizmo_size * 1.62
            self.place_gizmo_controls(anchor, center, None)

        def cut_face_size(self):
            """计算切割面（贴合面）的长宽，返回 (w, h) 毫米或 None，宽≥高。"""
            if self.m.working is None or self.m.mount_point is None or self.m.mount_normal is None:
                return None
            try:
                normal = np.asarray(self.m.mount_normal, float)
                normal /= max(np.linalg.norm(normal), 1e-12)
                point = self.m.mount_point - normal * self.depth.value()
                verts = np.asarray(self.m.working.vertices, float)
                # 平面内的两个正交投影轴（顺序不影响结果，最后统一宽≥高）。
                u = perpendicular_component(np.array([0.0, 0.0, 1.0]), normal)
                if u is None:
                    u = perpendicular_component(np.array([0.0, 1.0, 0.0]), normal)
                v = np.cross(normal, u)

                def extents(points):
                    rel = np.asarray(points, float) - point
                    uu, vv = rel @ u, rel @ v
                    w, h = float(uu.max() - uu.min()), float(vv.max() - vv.min())
                    return max(w, h), min(w, h)

                # 1) 共面三角形：切割封口/平面贴合面的三角形顶点都在平面内，
                #    直接取轮廓，避免 mesh_plane 对“平面与表面共面”返回空。
                faces = np.asarray(self.m.working.faces, int)
                dist = np.abs((verts - point) @ normal)
                scale = float(np.abs(verts).max()) if len(verts) else 1.0
                tol = 1e-5 * max(scale, 1.0)
                on_face = (dist[faces] < tol).all(axis=1)
                if on_face.any():
                    return extents(verts[np.unique(faces[on_face])])
                # 2) 斜切等穿过网格的平面：用 mesh_plane 截面轮廓。
                from trimesh.intersections import mesh_plane
                lines = mesh_plane(self.m.working, normal, point)
                if lines is not None and len(lines):
                    return extents(np.asarray(lines, float).reshape(-1, 3))
                return None
            except Exception:
                return None

        def show_ruler(self):
            """自动在查看器底部显示切割面长宽（选好贴合面/切割后触发）。"""
            self.hide_ruler()
            size = self.cut_face_size()
            if size is None:
                self.ruler_active = False
                return
            w, h = size
            self.ruler_actor = self.plot.add_text(
                f"切割面  宽 {w:.1f} × 高 {h:.1f} mm",
                position="lower_edge", font_size=13, color="#c0392b")
            self.plot.render()
            self.info.setText(f"切割面尺寸：{w:.2f} × {h:.2f} mm")

        def hide_ruler(self):
            if self.ruler_actor is not None:
                try: self.plot.remove_actor(self.ruler_actor)
                except Exception: pass
                self.ruler_actor = None
                self.plot.render()

        def enable_picking(self, enabled=True):
            """进入主体面拾取模式；实际拾取统一由自定义 VTK style 执行。"""
            self.pick_enabled = enabled
            if enabled:
                self.hook_select_enabled = False
            try:
                self.plot.disable_picking()
            except Exception as exc:
                self.log(f"disable_picking: {exc}")
            if enabled:
                self.log("已启用面拾取：请在模型表面单击")

        def stop_picking(self):
            self.pick_enabled = False
            try: self.plot.disable_picking()
            except Exception: pass

        def enable_hook_selection(self):
            self.hook_select_enabled = True
            self.pick_enabled = False
            try:
                self.plot.disable_picking()
                self.log("背钩可直接点击选择")
            except Exception as exc:
                self.log(f"启用背钩选择失败：{exc}")

        def clear_hook_selection(self):
            """Esc 是唯一的取消选择动作；点击空白只关闭操纵器。"""
            if self.gizmo_visible:
                self.hide_gizmo()
            if not self.m.selected_hook_indices:
                return
            self.m.selected_hook_indices.clear()
            self.gizmo_drag = None; self.gizmo_dragging = False
            self.refresh(preserve_camera=True)
            self.info.setText("已取消背钩选择；点击背钩重新选择")
            self.log("已按 Esc 取消背钩选择")

        def delete_selected_hooks(self):
            if not self.m.selected_hook_indices:
                return
            try:
                self.m.remember()
                self.m.delete_selected_hooks()
                if self.gizmo_visible:
                    self.hide_gizmo()
                self.refresh(preserve_camera=True)
                self.info.setText("已删除所选背钩")
                self.record_operation("删除所选背钩")
            except Exception as exc:
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "删除失败", str(exc))

        def refresh(self, preserve_camera=False):
            camera_position = self.plot.camera_position if preserve_camera else None
            self.plot.clear()
            self.body_actor = None; self.body_actor_key = None
            if self.m.working is not None:
                self.body_actor = self.plot.add_mesh(pv.wrap(self.m.working), color="#626a71", name="body",
                                   smooth_shading=True, ambient=0.18, diffuse=0.78,
                                   specular=0.20, specular_power=22)
                self.body_actor_key = self.actor_key(self.body_actor)
            self.hook_actors = {}; self.hook_actor_keys = {}
            self.hook_actor = None
            for index, part in enumerate(self.m.hook_parts):
                selected = index in self.m.selected_hook_indices
                # 选中用纯色高亮（亮琥珀色），不显示浅黄边线。
                actor = self.plot.add_mesh(pv.wrap(part), color="#ffb300" if selected else "#77818b",
                                           name=f"hook_{index}", smooth_shading=True, ambient=0.30,
                                           diffuse=0.80, specular=0.35 if selected else 0.06, specular_power=28)
                actor.GetProperty().SetEdgeVisibility(False)
                self.hook_actors[actor] = index
                self.hook_actor_keys[self.actor_key(actor)] = index
                if selected:
                    self.hook_actor = actor
            if self.m.mount_point is not None:
                self.plot.add_mesh(pv.PolyData(self.m.mount_point), color="red", point_size=14,
                                   render_points_as_spheres=True, name="mount_marker", pickable=False)
            if camera_position is not None:
                self.plot.camera_position = camera_position
            else:
                self.plot.reset_camera()
            self.plot.render()
            if self.gizmo_visible:
                self.build_gizmo()
                if self.gizmo_active is not None:
                    self.update_gizmo_editor(self.gizmo_active[0], self.gizmo_active[1])
            elif self.pick_enabled:
                self.enable_picking(True)
            elif self.hook_select_enabled or self.m.hook_parts:
                # 刷新会重建 VTK actor；只要不是“选择贴合面”或“正编辑手柄”，
                # 钩子就必须回到可点击状态，不能遗留一个没有任何选择入口的状态。
                self.enable_hook_selection()
            self.update_offset_readout()
            if self.ruler_active:
                self.show_ruler()

        @staticmethod
        def actor_key(actor):
            return actor.GetAddressAsString("") if actor is not None else None

        def hook_index_for_actor(self, actor):
            return self.hook_actor_keys.get(self.actor_key(actor))

        def set_view(self, view_function):
            """保留给等轴测等非命名视图的调用。"""
            view_function()
            self.plot.reset_camera()
            self.plot.render()
            self.log("已切换标准视图；模型原始坐标决定前/后/上/右方向")

        def set_named_view(self, face):
            """按固定世界轴建立真正的正交六视图，避免相机残余 roll 造成偏斜。"""
            views = {
                "front": ((0., -1., 0.), (0., 0., 1.)),
                "back": ((0., 1., 0.), (0., 0., 1.)),
                "right": ((1., 0., 0.), (0., 0., 1.)),
                "left": ((-1., 0., 0.), (0., 0., 1.)),
                "top": ((0., 0., 1.), (0., 1., 0.)),
                "bottom": ((0., 0., -1.), (0., -1., 0.)),
            }
            if face not in views:
                return
            camera = self.plot.renderer.GetActiveCamera()
            focal = np.asarray(camera.GetFocalPoint(), dtype=float)
            position = np.asarray(camera.GetPosition(), dtype=float)
            distance = max(float(np.linalg.norm(position - focal)), 1.0)
            direction, up = views[face]
            camera.SetFocalPoint(*focal)
            camera.SetPosition(*(focal + np.asarray(direction) * distance))
            camera.SetViewUp(*up)
            camera.ParallelProjectionOn()
            self.plot.reset_camera()
            # reset_camera 可能为了适配窗口调整距离，但不会改变我们刚定义的方向。
            camera.SetViewUp(*up); camera.OrthogonalizeViewUp()
            self.plot.render()
            self.log(f"已切换严格正交{face}视图")

        def picked(self, point, picker):
            cell_id = picker.GetCellId()
            data = picker.GetDataSet()
            actor = picker.GetActor()
            if self.hook_select_enabled and actor in self.hook_actors:
                self.m.select_hook(self.hook_actors[actor])
                self.hook_actor = actor
                self.log(f"已点击选中背钩 #{self.m.selected_hook_index + 1}，显示三轴操纵器")
                self.show_gizmo()
                return
            if not self.pick_enabled:
                return
            self.log(f"收到拾取回调：cell={cell_id} point={np.asarray(point).round(3).tolist()}")
            if data is None or cell_id < 0:
                self.info.setText("没有拾取到面，请点击主体模型表面")
                self.log("未拾取到三角面（可能点到了空白处）")
                return
            try:
                mesh = pv.wrap(data)
                mesh.compute_normals(cell_normals=True, point_normals=False, inplace=True)
                normal = np.asarray(mesh.cell_data["Normals"][cell_id], float)
                point = np.asarray(point, float)
                self.m.remember()
                self.m.set_mount_face(point, normal)
                self.stop_picking()
                self.plot.add_mesh(pv.PolyData(point), color="red", point_size=14,
                                   render_points_as_spheres=True, name="mount_marker", pickable=False)
                self.info.setText("已选择贴合面：" + str(point.round(2)))
                self.log(f"贴合面 point={point.round(3).tolist()} normal={normal.round(3).tolist()}")
                self.record_operation("选择贴合面")
                self.ruler_active = True
                self.show_ruler()
            except Exception as exc:
                self.info.setText("拾取到了对象，但无法计算面法线")
                self.log(f"处理拾取失败：{type(exc).__name__}: {exc}")

        def pick_mount_face_at(self, pos):
            """从统一的左键分发器拾取主体三角面，不依赖 PyVista 回调。"""
            if self.body_actor is None:
                self.info.setText("请先导入主体模型")
                return False
            picker = self.body_picker
            picker.InitializePickList(); picker.AddPickList(self.body_actor); picker.PickFromListOn()
            picker.Pick(pos[0], pos[1], 0, self.plot.renderer)
            actor = picker.GetActor(); cell_id = picker.GetCellId(); data = picker.GetDataSet()
            if self.actor_key(actor) != self.body_actor_key or data is None or cell_id < 0:
                self.info.setText("没有拾取到主体面，请点击主体模型表面")
                self.log("主体面拾取未命中")
                return False
            try:
                mesh = pv.wrap(data)
                mesh.compute_normals(cell_normals=True, point_normals=False, inplace=True)
                normal = np.asarray(mesh.cell_data["Normals"][cell_id], float)
                point = np.asarray(picker.GetPickPosition(), float)
                self.m.remember(); self.m.set_mount_face(point, normal)
                self.stop_picking()
                self.plot.add_mesh(pv.PolyData(point), color="red", point_size=14,
                                   render_points_as_spheres=True, name="mount_marker", pickable=False)
                self.info.setText("已选择贴合面：" + str(point.round(2)))
                self.log(f"贴合面 point={point.round(3).tolist()} normal={normal.round(3).tolist()}")
                self.record_operation("选择贴合面")
                self.plot.render()
                self.ruler_active = True
                self.show_ruler()
                return True
            except Exception as exc:
                self.info.setText("拾取到了主体，但无法计算面法线")
                self.log(f"处理主体面拾取失败：{type(exc).__name__}: {exc}")
                return False

        def select_hook_at(self, pos, show_gizmo=False):
            """拾取独立钩子 actor，并在普通模式下直接进入编辑操纵器。"""
            if not self.hook_actors:
                return False
            picker = self.hook_picker
            picker.InitializePickList()
            for actor in self.hook_actors:
                picker.AddPickList(actor)
            picker.PickFromListOn(); picker.Pick(pos[0], pos[1], 0, self.plot.renderer)
            actor = picker.GetActor()
            index = self.hook_index_for_actor(actor)
            if index is None:
                self.log("背钩拾取未命中：VTK actor 不在当前背钩映射中")
                return False
            shift = bool(self.plot.iren.interactor.GetShiftKey())
            control = bool(self.plot.iren.interactor.GetControlKey())
            command = bool(QtWidgets.QApplication.keyboardModifiers() & QtCore.Qt.MetaModifier)
            self.m.select_hook(index, toggle=(shift or control or command))
            self.hook_actor = actor
            self.refresh(preserve_camera=True)
            self.record_operation(f"选择背钩 {[i + 1 for i in sorted(self.m.selected_hook_indices)]}")
            if show_gizmo:
                self.log(f"已点击选中背钩 #{index + 1}，显示三轴操纵器")
                self.show_gizmo()
            else:
                self.plot.render()
            return True

        def load_body(self):
            p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "主体模型", "", "3D 模型 (*.stl *.step *.stp *.3mf);;STL (*.stl);;STEP (*.step *.stp);;3MF (*.3mf)")
            if p:
                self.m.remember(); self.m.set_body(p)
                # 主体更换后旧贴合面失效，隐藏自动尺寸标注。
                self.ruler_active = False
                self.hide_ruler()
                self.refresh(); self.info.setText(Path(p).name)
                self.record_operation("导入主体：" + Path(p).name)
        def load_hook(self):
            p, _ = QtWidgets.QFileDialog.getOpenFileName(self, "替换背钩模型", "", "3D 模型 (*.stl *.step *.stp *.3mf);;STL (*.stl);;STEP (*.step *.stp);;3MF (*.3mf)")
            if p:
                if self.gizmo_visible: self.hide_gizmo()
                self.m.remember(); self.m.set_hook(p)
                self.reset_align_state()
                self.record_operation("手动导入替换背钩：" + Path(p).name)
                if self.m.mount_point is not None:
                    self.m.align_hook()
                    self.record_operation("背钩自动贴合到所选面")
                self.refresh(); self.enable_hook_selection(); self.info.setText(Path(p).name)
                self.log("背钩导入完成：可直接单击橙色背钩进入编辑")

        def load_hook_preset(self, _index):
            key = self.hook_preset.currentData()
            if key is None:
                return
            path = self.preset_paths[key]
            if path is None or not path.exists():
                QtWidgets.QMessageBox.information(self, "预设未安装", f"内置 {key} 背钩尚未提供 STL 文件。请使用“手动导入背钩”。")
                self.hook_preset.setCurrentIndex(0)
                return
            if self.gizmo_visible:
                self.hide_gizmo()
            self.m.remember(); self.m.set_hook(path)
            self.reset_align_state()
            self.info.setText(f"已载入内置 {key} 背钩")
            self.record_operation(f"载入内置 {key} 背钩")
            if self.m.mount_point is not None:
                self.m.align_hook()
                self.record_operation("背钩自动贴合到所选面")
            self.refresh(); self.enable_hook_selection()
            self.log("内置背钩载入完成：可直接单击橙色背钩进入编辑")

        def schedule_side_transform_preview(self, _value=0.0):
            if self._side_transform_updating or not self.m.hook_parts:
                return
            self._side_transform_timer.start(180)

        def preview_side_transform(self):
            if self._side_transform_updating or not self.m.hook_parts:
                return
            values = {key: field.value() for key, field in self.transform_fields.items()}
            if self._side_transform_session is None:
                if not any(abs(value) > 1e-12 for value in values.values()):
                    return
                indices = self.m.editable_hook_indices()
                self.m.remember()
                self._side_transform_session = {
                    "indices": indices,
                    "base_parts": [self.m.hook_parts[index].copy() for index in indices],
                    "dirty": False,
                }
            session = self._side_transform_session
            try:
                # 每次预览都回到输入开始时的网格，再按当前完整输入量计算，
                # 例如把 1 改成 10 时只得到 +10 mm，而不是 +1 再 +10。
                for index, source in zip(session["indices"], session["base_parts"]):
                    self.m.hook_parts[index] = source.copy()
                self.m.rebuild_hook()
                self.m.transform_hook((values["tx"], values["ty"], values["tz"]),
                                      (values["rx"], values["ry"], values["rz"]), False)
                session["dirty"] = any(abs(value) > 1e-12 for value in values.values())
                self.refresh(preserve_camera=True)
                self.info.setText("侧栏数值已实时预览")
            except Exception as exc:
                if self.m.history:
                    self.m.history.pop()
                self._side_transform_session = None
                QtWidgets.QMessageBox.warning(self, "变换失败", str(exc))

        def finish_side_transform(self):
            if self._side_transform_updating:
                return
            self._side_transform_timer.stop()
            self.preview_side_transform()
            session = self._side_transform_session
            if session is None:
                return
            if session["dirty"]:
                self.record_operation("侧栏精确变换")
            else:
                if self.m.history:
                    self.m.history.pop()
            self._side_transform_session = None
            self._side_transform_updating = True
            try:
                for field in self.transform_fields.values():
                    field.setValue(0.0)
            finally:
                self._side_transform_updating = False

        def apply_side_transform(self):
            """兼容旧连接：侧栏现在由实时预览和编辑完成事件处理。"""
            self.finish_side_transform()

        def combine_selected_hooks(self):
            try:
                self.m.remember(); self.m.combine_selected(); self.refresh(preserve_camera=True)
                self.info.setText(f"已组合 {len(self.m.selected_hook_indices)} 个背钩")
                self.record_operation("组合背钩：" + str([i + 1 for i in sorted(self.m.selected_hook_indices)]))
            except Exception as exc:
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "组合失败", str(exc))

        def ungroup_selected_hooks(self):
            try:
                self.m.remember(); self.m.ungroup_selected(); self.refresh(preserve_camera=True)
                self.info.setText("已解散所选组合")
                self.record_operation("解散所选组合")
            except Exception as exc:
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "解散失败", str(exc))
        def pick_mode(self):
            if self.m.working is None:
                self.info.setText("请先导入主体模型")
                return
            if self.gizmo_visible:
                self.hide_gizmo()
            self.info.setText("请在模型上点击主体贴合面")
            self.enable_picking(True)

        # ------------------------------------------------------------------
        # 三轴箭头（平移）+ 三轴圆环（旋转）操纵器。
        #
        # 关键原则：增量矩阵永远由 translation_matrix(delta) 或
        # rotation_matrix(angle, axis, point=center) 直接构造，不从任何
        # widget 的内部矩阵反解——这两个函数保证输出是纯刚体变换，不可能
        # 带入缩放/剪切，从根源上避免了旧 vtkBoxWidget2 方案里出现的
        # 压扁、飞走、松手后仍跟随的问题。
        # ------------------------------------------------------------------

        def toggle_gizmo(self):
            if self.gizmo_visible:
                self.hide_gizmo()
            else:
                self.show_gizmo()

        def show_gizmo(self):
            if self.m.hook is None:
                self.info.setText("请先导入替换背钩模型")
                return
            if not self.m.selected_hook_indices:
                self.info.setText("请先点击选择一个背钩")
                return
            self.stop_picking()
            self.hook_select_enabled = False
            self.gizmo_visible = True
            self.build_gizmo()
            self.plot.iren.interactor.SetInteractorStyle(self.gizmo_style)
            self.plot.render()
            self.info.setText("拖动箭头平移、拖动圆环旋转；勾选拷贝可复制拖动；点击空白收起手柄，Esc 取消选择")
            self.log("三轴操纵器已启用")
            self.set_gizmo_active("translate", "x", np.array([1.0, 0.0, 0.0]))

        def set_gizmo_active(self, mode, axis_name, axis):
            """记录当前活跃轴并刷新常驻悬浮框。"""
            self.gizmo_active = (mode, axis_name, axis)
            self.update_gizmo_editor(mode, axis_name)

        def update_gizmo_editor(self, mode, axis_name):
            """按活跃轴刷新悬浮框徽章/单位，归零数值，贴到对应手柄外侧。"""
            if self.gizmo_editor is None:
                return
            axis_color = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}[axis_name]
            self.gizmo_axis_badge.setText(axis_name.upper())
            self.gizmo_axis_badge.setStyleSheet(f"background:{axis_color}; color:white; border-radius:3px; font-weight:bold;")
            self.gizmo_value.setSuffix(" mm" if mode == "translate" else " °")
            self._set_value_silent(0.0)
            self.gizmo_editor.show()
            self.place_gizmo_editor()

        def place_gizmo_editor(self):
            """把悬浮框放到当前活跃轴手柄外侧（箭头尖端/圆环法线外侧）。"""
            if self.gizmo_center is None or self.gizmo_active is None:
                return
            _mode, _name, axis = self.gizmo_active
            anchor = self.gizmo_center + axis * self.gizmo_size * 1.62
            self.place_gizmo_controls(anchor, self.gizmo_center, None)

        def _set_value_silent(self, value):
            self.gizmo_value.blockSignals(True)
            self.gizmo_value.setValue(value)
            self.gizmo_value.blockSignals(False)

        def hide_gizmo(self):
            self.gizmo_visible = False
            self.gizmo_dragging = False; self.gizmo_drag = None
            self.hide_gizmo_editor()
            self.clear_gizmo_actors()
            # 自定义 style 始终保留：它既负责普通相机，也负责左下角导航立方体
            # 的事件路由；切回 default_style 会导致立方体再次“只能看不能点”。
            self.plot.iren.interactor.SetInteractorStyle(self.gizmo_style)
            self.hook_select_enabled = True
            self.enable_hook_selection()
            self.plot.render()
            self.log("三轴操纵器已关闭；当前背钩仍保持选中，按 Esc 才取消选择")

        def clear_gizmo_actors(self):
            for actor in list(self.gizmo_actor_info):
                try: self.plot.remove_actor(actor)
                except Exception: pass
            self.gizmo_actor_info = {}
            self.gizmo_base_colors = {}
            self.gizmo_hovered = None

        def build_gizmo(self):
            """在钩子当前质心处重建 6 个手柄；每次提交变换后都会调用一次，
            所以手柄总是跟随钩子的最新位置——这也保证了钩子始终是独立、
            可反复编辑的对象，而不是贴合后就和主体焊死。"""
            self.clear_gizmo_actors()
            if not self.m.hook_parts:
                return
            edit_parts = [self.m.hook_parts[i] for i in self.m.editable_hook_indices()]
            if not edit_parts:
                return
            selected_mesh = trimesh.util.concatenate(edit_parts)
            center = np.asarray(selected_mesh.centroid, float)
            bounds_diag = float(np.linalg.norm(selected_mesh.bounds[1] - selected_mesh.bounds[0]))
            size = max(bounds_diag * 0.6, 5.0)
            self.gizmo_center = center
            self.gizmo_size = size
            for (mode, axis_name), (mesh, color, axis) in build_gizmo_geometry(pv, center, size).items():
                actor = self.plot.add_mesh(mesh, color=color, name=f"gizmo_{mode}_{axis_name}",
                                            pickable=True, opacity=1.0 if mode == "translate" else 0.85)
                actor.GetProperty().SetAmbient(0.6); actor.GetProperty().SetDiffuse(0.6)
                self.gizmo_actor_info[actor] = (mode, axis_name, axis)
                self.gizmo_base_colors[actor] = color
            self.gizmo_picker.InitializePickList()
            for actor in self.gizmo_actor_info:
                self.gizmo_picker.AddPickList(actor)
            self.gizmo_picker.PickFromListOn()

        def gizmo_try_select_hook(self, pos):
            """操纵器显示时仍可直接点另一个钩子切换编辑对象。"""
            if not self.gizmo_visible or not self.hook_actors:
                return False
            self.hook_picker.InitializePickList()
            for actor in self.hook_actors:
                self.hook_picker.AddPickList(actor)
            self.hook_picker.PickFromListOn()
            self.hook_picker.Pick(pos[0], pos[1], 0, self.plot.renderer)
            actor = self.hook_picker.GetActor()
            index = self.hook_index_for_actor(actor)
            if index is None:
                return False
            if self.gizmo_drag is not None and not self.gizmo_dragging and not self.gizmo_drag["model_dirty"]:
                if self.m.history:
                    self.m.history.pop()
                self.gizmo_drag = None
                self.hide_gizmo_editor()
            shift = bool(self.plot.iren.interactor.GetShiftKey())
            control = bool(self.plot.iren.interactor.GetControlKey())
            command = bool(QtWidgets.QApplication.keyboardModifiers() & QtCore.Qt.MetaModifier)
            self.m.select_hook(index, toggle=(shift or control or command))
            self.hook_actor = actor
            self.refresh(preserve_camera=True)
            self.plot.render()
            self.record_operation(f"选择背钩 {[i + 1 for i in sorted(self.m.selected_hook_indices)]}")
            return True

        def reset_handle_colors(self):
            for actor, color in self.gizmo_base_colors.items():
                r, g, b = pv.Color(color).float_rgb
                actor.GetProperty().SetColor(r, g, b)
            self.gizmo_hovered = None

        def highlight_handle(self, actor):
            self.reset_handle_colors()
            actor.GetProperty().SetColor(1.0, 0.85, 0.0)
            self.gizmo_hovered = actor

        def gizmo_update_hover(self, pos):
            if not self.gizmo_visible or self.gizmo_dragging or not self.gizmo_actor_info:
                return
            self.gizmo_picker.Pick(pos[0], pos[1], 0, self.plot.renderer)
            actor = self.gizmo_picker.GetActor()
            if actor is self.gizmo_hovered:
                return
            if actor in self.gizmo_actor_info:
                self.highlight_handle(actor)
            else:
                self.reset_handle_colors()
            self.plot.render()

        def gizmo_try_start_drag(self, pos):
            if not self.gizmo_visible or not self.gizmo_actor_info:
                return False
            # 仅点击一个轴后会留下等待数值输入的上下文。点另一个轴时丢弃这条
            # 尚未改动网格的历史记录，再开启新的轴输入。
            if self.gizmo_drag is not None and not self.gizmo_dragging:
                if not self.gizmo_drag["model_dirty"] and self.m.history:
                    self.m.history.pop()
                self.gizmo_drag = None
                self.hide_gizmo_editor()
            self.gizmo_picker.Pick(pos[0], pos[1], 0, self.plot.renderer)
            actor = self.gizmo_picker.GetActor()
            info = self.gizmo_actor_info.get(actor)
            if info is None:
                return False
            mode, axis_name, axis = info
            origin, direction = screen_ray(self.plot.renderer, *pos)
            center = self.gizmo_center.copy()
            self.m.remember()
            # 真实网格在每一帧移动。不要把 vtk Actor 的 UserTransform 当作
            # 编辑状态：它仅是显示层，actor 重建后会自然丢失，正是此前松手回弹的来源。
            source_indices = self.m.editable_hook_indices()
            sources = [self.m.hook_parts[index].copy() for index in source_indices]
            drag = {"mode": mode, "axis_name": axis_name, "axis": axis, "center": center,
                    "copy": self.gizmo_copy_box.isChecked(), "matrix": np.eye(4),
                    "base_parts": sources, "source_indices": source_indices, "last_value": 0.0,
                    "model_dirty": False, "copy_inserted": False, "target_indices": source_indices.copy()}
            if mode == "translate":
                tc = axis_param_from_ray(origin, direction, center, axis)
                if tc is None:
                    if self.m.history: self.m.history.pop()
                    return False
                drag["start"] = tc
            else:
                q = ray_plane_point(origin, direction, center, axis)
                ref = perpendicular_component(q - center, axis) if q is not None else None
                if ref is None:
                    if self.m.history: self.m.history.pop()
                    return False
                drag["ref"] = ref
            self.gizmo_drag = drag
            self.gizmo_dragging = True
            self.highlight_handle(actor)
            self.set_gizmo_active(mode, axis_name, axis)
            self.plot.render()
            return True

        def place_gizmo_controls(self, world_anchor, gizmo_center, fallback_display):
            """工具条始终在箭头外侧，避免盖住钩子或主体。"""
            if world_anchor is not None:
                self.plot.renderer.SetWorldPoint(*[float(v) for v in world_anchor], 1.0)
                self.plot.renderer.WorldToDisplay()
                display = self.plot.renderer.GetDisplayPoint()
                self.plot.renderer.SetWorldPoint(*[float(v) for v in gizmo_center], 1.0)
                self.plot.renderer.WorldToDisplay()
                center_display = self.plot.renderer.GetDisplayPoint()
            else:
                display = fallback_display
                center_display = (display[0] - 1, display[1] - 1)
            render_width, render_height = self.plot.renderer.GetSize()
            scale_x = self.plot.width() / max(render_width, 1)
            scale_y = self.plot.height() / max(render_height, 1)
            self.gizmo_editor.adjustSize()
            total_width = self.gizmo_editor.width()
            total_height = self.gizmo_editor.height()
            dx = display[0] - center_display[0]
            dy = display[1] - center_display[1]
            # 朝箭头离开钩子的方向排布，而不是固定落在箭头上方。
            x = int(display[0] * scale_x + (14 if dx >= 0 else -total_width - 14))
            y = int((render_height - display[1]) * scale_y + (-total_height - 10 if dy >= 0 else 10))
            x = max(6, min(x, self.plot.width() - total_width - 6))
            y = max(6, min(y, self.plot.height() - total_height - 6))
            self.gizmo_editor.move(x, y)

        def hide_gizmo_editor(self):
            if self.gizmo_editor is not None:
                self.gizmo_editor.hide()

        def apply_gizmo_value(self):
            if self.gizmo_commit_lock or self.gizmo_active is None:
                return
            mode, axis_name, axis = self.gizmo_active
            value = float(self.gizmo_value.value())
            if abs(value) < 1e-12:
                return
            if not self.m.editable_hook_indices():
                return
            if mode == "translate":
                matrix = trimesh.transformations.translation_matrix(axis * value)
            else:
                matrix = trimesh.transformations.rotation_matrix(np.radians(value), axis, point=self.gizmo_center)
            self.gizmo_commit_lock = True
            try:
                self.m.remember()
                self.m.apply_hook_matrix(matrix, copy=self.gizmo_copy_box.isChecked())
                self.record_operation(f"{axis_name.upper()} 轴{'平移' if mode == 'translate' else '旋转'} {value:+.3f}{' mm' if mode == 'translate' else '°'}")
                self.refresh(preserve_camera=True)
                self.info.setText(f"背钩已{'平移' if mode == 'translate' else '旋转'}" + ("（已拷贝）" if self.gizmo_copy_box.isChecked() else ""))
            except Exception as exc:
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "变换失败", str(exc))
            finally:
                self.gizmo_commit_lock = False

        def gizmo_update_drag(self, pos):
            drag = self.gizmo_drag
            if drag is None or self.hook_actor is None:
                return
            origin, direction = screen_ray(self.plot.renderer, *pos)
            if drag["mode"] == "translate":
                tc = axis_param_from_ray(origin, direction, drag["center"], drag["axis"])
                if tc is None: return
                delta = (tc - drag["start"]) * drag["axis"]
                matrix = trimesh.transformations.translation_matrix(delta)
                value = float(tc - drag["start"])
            else:
                q = ray_plane_point(origin, direction, drag["center"], drag["axis"])
                v = perpendicular_component(q - drag["center"], drag["axis"]) if q is not None else None
                if v is None: return
                angle = signed_angle(drag["ref"], v, drag["axis"])
                matrix = trimesh.transformations.rotation_matrix(angle, drag["axis"], point=drag["center"])
                value = float(np.degrees(angle))
            # 直接拖动也读取同一个“拷贝”开关，不能只在点“应用”时才生效。
            drag["copy"] = self.gizmo_copy_box.isChecked()
            drag["matrix"] = matrix
            self.apply_drag_to_model(drag, value)
            self._set_value_silent(value)
            self.follow_gizmo_editor()   # 弹窗实时跟随移动后的钩子
            self.update_offset_readout() # 偏移读数实时刷新
            self.plot.render()

        def gizmo_finish_drag(self):
            if self.gizmo_commit_lock:
                return
            drag = self.gizmo_drag
            self.gizmo_dragging = False
            if drag is None:
                return
            if not drag["model_dirty"]:
                # 单击手柄不是一次零位移操作，而是打开这个方向的精确输入。
                # 保留 drag 上下文，使“输入正/负数 + 应用”可直接使用。
                self.gizmo_drag = drag
                self.log(f"已选择 {drag['axis_name'].upper()} {'平移' if drag['mode'] == 'translate' else '旋转'}；可输入精确数值或直接再次拖动")
                return
            self.gizmo_drag = None
            try:
                self.gizmo_commit_lock = True
                # 正常拖动已逐帧写入网格；没有收到 MouseMove 时才提交单位矩阵，
                # 这会保持原状且不产生一次空变换。
                if not drag["model_dirty"]:
                    self.apply_drag_to_model(drag, 0.0)
                self.finish_drag_visuals(drag)
            except Exception as exc:
                if drag["model_dirty"]:
                    self.m.undo()
                self.reset_handle_colors(); self.plot.render()
                self.log(f"gizmo 提交失败：{type(exc).__name__}: {exc}")
            finally:
                self.gizmo_commit_lock = False

        def apply_drag_to_model(self, drag, value):
            """以起拖时的网格为基准，写入当前增量并更新同一个 actor。

            这里不调用 refresh()，避免拖动时清空 renderer、重设相机或使鼠标事件
            指向一个已经销毁的 actor。每一帧都是刚体变换后的真实 STL 数据。
            """
            matrix = np.asarray(drag["matrix"], dtype=float)
            targets = [source.copy() for source in drag["base_parts"]]
            for target in targets:
                target.apply_transform(matrix)
            if drag["copy"]:
                if not drag["copy_inserted"]:
                    start = len(self.m.hook_parts)
                    self.m.hook_parts.extend(targets)
                    drag["copy_inserted"] = True
                    drag["target_indices"] = list(range(start, start + len(targets)))
                    self.m.selected_hook_indices = set(drag["target_indices"])
                    self.m.selected_hook_index = start
                    if len(targets) > 1:
                        self.m.hook_groups.append(set(drag["target_indices"]))
                else:
                    for index, target in zip(drag["target_indices"], targets):
                        self.m.hook_parts[index] = target
            else:
                for index, target in zip(drag["target_indices"], targets):
                    self.m.hook_parts[index] = target
            self.m.rebuild_hook()
            drag["model_dirty"] = True
            drag["last_value"] = value
            if self.hook_actor is not None:
                for index, target in zip(drag["target_indices"], targets):
                    for actor, actor_index in self.hook_actors.items():
                        if actor_index == index:
                            actor.GetMapper().SetInputData(pv.wrap(target)); actor.GetMapper().Modified(); actor.Modified()

        def finish_drag_visuals(self, drag):
            matrix = np.asarray(drag.get("matrix", np.eye(4)), dtype=float)
            self.record_operation(f"{drag['axis_name'].upper()} 轴{'平移' if drag['mode'] == 'translate' else '旋转'} {drag['last_value']:+.3f}{' mm' if drag['mode'] == 'translate' else '°'}")
            # 松手后才重建 gizmo；保持相机，不再 reset_camera 造成视觉跳动。
            self.refresh(preserve_camera=True)
            mode_label = "平移" if drag["mode"] == "translate" else "旋转"
            self.info.setText(f"背钩已{mode_label}" + ("（已拷贝）" if drag["copy"] else ""))

        def cut(self):
            try:
                self.m.remember()
                report = self.m.cut_by_plane(self.depth.value())
                self.refresh(); self.info.setText("已保留主体侧，已切除较小的外伸侧")
                self.record_operation("切除外侧背钩")
            except Exception as e:
                # 切割若失败，不占用一条撤销记录。
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "切除失败", str(e))
        def align(self):
            if self.aligned:
                self.cancel_align()
                return
            try:
                self._pre_align_state = {
                    "parts": [part.copy() for part in self.m.hook_parts],
                    "groups": [set(group) for group in self.m.hook_groups],
                    "selection": set(self.m.selected_hook_indices),
                    "index": self.m.selected_hook_index,
                }
                self.m.remember()
                self.m.align_hook()
                indices = self.m.editable_hook_indices()
                self.gizmo_offset_origin = trimesh.util.concatenate(
                    [self.m.hook_parts[i] for i in indices]).centroid.copy() if indices else None
                self.aligned = True
                self.align_button.setText("取消贴合")
                self.align_button.setStyleSheet("background:#c0392b; color:white; font-weight:bold; padding:7px;")
                self.refresh()
                self.update_offset_readout()
                self.info.setText("背钩已贴合（偏移原点归零）；再次点击按钮可取消贴合")
                self.record_operation("背钩贴合到所选面")
            except Exception as e:
                if self.m.history: self.m.history.pop()
                QtWidgets.QMessageBox.warning(self, "贴合失败", str(e))

        def cancel_align(self):
            state = self._pre_align_state
            if state is None:
                return
            self.m.hook_parts = [part.copy() for part in state["parts"]]
            self.m.hook_groups = [set(group) for group in state["groups"]]
            self.m.selected_hook_indices = set(state["selection"])
            self.m.selected_hook_index = state["index"]
            self.m.rebuild_hook()
            self.gizmo_offset_origin = None
            self.aligned = False
            self._pre_align_state = None
            self.align_button.setText("背钩贴合")
            self.align_button.setStyleSheet("background:#2d9d5a; color:white; font-weight:bold; padding:7px;")
            if self.gizmo_visible:
                self.hide_gizmo()
            self.refresh()
            self.update_offset_readout()
            self.info.setText("已取消贴合，背钩回到贴合前位置")
            self.record_operation("取消贴合")

        def undo(self):
            self.finish_side_transform()
            if self.m.undo():
                self.refresh(); self.info.setText(f"已撤销；剩余可撤销操作：{len(self.m.history)}")
                self.record_operation("撤销上一步")
            else:
                self.info.setText("没有可撤销的操作")

        def redo(self):
            self.finish_side_transform()
            if self.m.redo():
                self.refresh(); self.info.setText(f"已重做；剩余可重做操作：{len(self.m.redo_history)}")
                self.record_operation("重做上一步")
            else:
                self.info.setText("没有可重做的操作")
        def save(self):
            p, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出 STL", "converted.stl", "STL (*.stl)")
            if p:
                try: self.m.export(p); self.info.setText("已导出：" + p)
                except Exception as e: QtWidgets.QMessageBox.warning(self, "导出失败", str(e))
    print("[启动] 创建 Qt 应用...")
    app = QtWidgets.QApplication(sys.argv)
    print("[启动] ✓ Qt 应用创建成功")
    print("[启动] 创建主窗口对象...")
    w = Window()
    print("[启动] ✓ 窗口对象创建完成")
    print("[启动] 显示窗口...")
    w.show()
    print("[启动] ✓ 窗口已显示")
    print("\n========================================")
    print("程序已启动！窗口应该已经显示。")
    print("========================================\n")
    return app.exec()


if __name__ == "__main__":
    try: raise SystemExit(run_gui())
    except ImportError as e: raise SystemExit("缺少依赖，请运行 pip install -r requirements.txt\n" + str(e))
