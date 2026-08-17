import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import sys
import re

SNAP = 10          # grid size in pixels
HIT_R = 8          # vertex hit radius for drag/delete
CLOSE_R = 8        # radius to close the current chain back onto its start
FILL_OUT = "#bcd9ff"
EDGE_OUT = "#1a4fc4"
EDGE_IN = "#d43b3b"
VERTEX = "#222222"
GRID = "#e8e8e8"
TEXT = "#888888"


def fmt(v):
    v = float(v)
    return str(int(v)) if v == int(v) else ("%g" % v)


def signed_area(pts):
    """
    Computes signed area of a 2D polygon in Cartesian coordinates (+y up).
    Positive area (> 0) means Counter-Clockwise (CCW).
    Negative area (< 0) means Clockwise (CW).
    """
    n = len(pts)
    if n < 3:
        return 0.0
    return 0.5 * sum(
        pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
        for i in range(n)
    )


def normalize_orientation(pts, ctype):
    """
    Ensures that:
    - 'out' (outer boundary) is strictly Counter-Clockwise (CCW, signed_area > 0)
    - 'in' (hole) is strictly Clockwise (CW, signed_area < 0)
    """
    if len(pts) < 3:
        return list(pts)
    area = signed_area(pts)
    if ctype == "out" and area < 0:
        return list(reversed(pts))
    elif ctype == "in" and area > 0:
        return list(reversed(pts))
    return list(pts)


class PolyEditor:
    def __init__(self, root, file_to_load=None):
        self.root = root
        root.title("ACD2D .poly Editor")
        root.geometry("960x700")

        self.chains = []          # committed chains: {'type':..., 'pts':[...]}
        self.cur_pts = []         # points of the chain being drawn
        self.cur_type = "out"     # type of the chain being drawn
        self.drag = None          # (chain_idx, pt_idx) being dragged
        self.snap_enabled = tk.BooleanVar(value=True)
        self.file_path = None

        # view transform: world (stored coords) -> screen
        self.scale = 1.0
        self.ox = 0.0
        self.oy = 0.0
        self.pan_start = None     # (sx, sy, ox, oy) for middle-button pan

        self._build_menu()
        self._build_toolbar()
        self._build_canvas()
        self._build_status()

        self.canvas.bind("<Button-1>", self.on_left_down)
        self.canvas.bind("<Double-Button-1>", self.on_double_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_left_up)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<Button-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan)
        self.canvas.bind("<ButtonRelease-2>", self.on_pan_end)
        root.bind("<Control-MouseWheel>", self.on_zoom)
        root.bind("<Control-Button-4>", self.on_zoom)
        root.bind("<Control-Button-5>", self.on_zoom)
        root.bind("<Control-s>", lambda e: self.save_dialog())
        root.bind("<Control-o>", lambda e: self.open_dialog())
        root.bind("<Control-n>", lambda e: self.new_file())
        root.bind("<Control-f>", lambda e: self.fit_view())
        root.bind("<Control-z>", lambda e: self.undo_point())
        root.bind("<Escape>", lambda e: self.close_chain())
        root.bind("<Delete>", lambda e: self.delete_last_chain())
        root.bind("<BackSpace>", lambda e: self.undo_point())

        if file_to_load:
            root.after(100, lambda: self.load_file(file_to_load))

        self.redraw()

    # ------------------------------------------------------------------ UI
    def _build_menu(self):
        menubar = tk.Menu(self.root)
        filemenu = tk.Menu(menubar, tearoff=0)
        filemenu.add_command(label="New", accelerator="Ctrl+N",
                             command=self.new_file)
        filemenu.add_command(label="Open...", accelerator="Ctrl+O",
                             command=self.open_dialog)
        filemenu.add_command(label="Save", accelerator="Ctrl+S",
                             command=self.save_dialog)
        filemenu.add_command(label="Save As...", command=self.save_as_dialog)
        filemenu.add_separator()
        filemenu.add_command(label="Quit", command=self.root.quit)
        menubar.add_cascade(label="File", menu=filemenu)

        editmenu = tk.Menu(menubar, tearoff=0)
        editmenu.add_command(label="Undo Point", accelerator="Ctrl+Z",
                             command=self.undo_point)
        editmenu.add_command(label="Close Chain", accelerator="Esc",
                             command=self.close_chain)
        editmenu.add_command(label="Delete Last Chain", accelerator="Del",
                             command=self.delete_last_chain)
        editmenu.add_separator()
        editmenu.add_command(label="Clear All", command=self.clear_all)
        menubar.add_cascade(label="Edit", menu=editmenu)

        viewmenu = tk.Menu(menubar, tearoff=0)
        viewmenu.add_command(label="Fit View", accelerator="Ctrl+F",
                             command=self.fit_view)
        viewmenu.add_command(label="Zoom In", command=lambda: self.zoom(1.2))
        viewmenu.add_command(label="Zoom Out", command=lambda: self.zoom(1.0 / 1.2))
        viewmenu.add_command(label="Reset Zoom (1:1)", command=self.reset_view)
        menubar.add_cascade(label="View", menu=viewmenu)

        helpmenu = tk.Menu(menubar, tearoff=0)
        helpmenu.add_command(label="Usage", command=self.show_usage)
        menubar.add_cascade(label="Help", menu=helpmenu)
        self.root.config(menu=menubar)

    def _build_toolbar(self):
        bar = ttk.Frame(self.root, padding=4)
        bar.pack(fill="x")

        self.type_var = tk.StringVar(value="out")
        ttk.Label(bar, text="Next chain:").pack(side="left")
        ttk.Radiobutton(bar, text="Outer", variable=self.type_var,
                        value="out", command=self.sync_type).pack(side="left")
        self.hole_rb = ttk.Radiobutton(bar, text="Hole", variable=self.type_var,
                                       value="in", command=self.sync_type)
        self.hole_rb.pack(side="left")

        ttk.Button(bar, text="New Chain", command=self.close_chain).pack(
            side="left", padx=4)
        ttk.Button(bar, text="Undo Point", command=self.undo_point).pack(
            side="left")
        ttk.Button(bar, text="Delete Last Chain",
                   command=self.delete_last_chain).pack(side="left")
        ttk.Checkbutton(bar, text="Snap", variable=self.snap_enabled,
                        command=self.redraw).pack(side="left", padx=6)
        ttk.Button(bar, text="Fit View", command=self.fit_view).pack(
            side="left", padx=2)

        ttk.Button(bar, text="Clear All", command=self.clear_all).pack(
            side="right")
        ttk.Button(bar, text="Save", command=self.save_dialog).pack(
            side="right", padx=4)
        ttk.Button(bar, text="Open...", command=self.open_dialog).pack(
            side="right")

    def _build_canvas(self):
        frame = ttk.Frame(self.root)
        frame.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(frame, bg="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.redraw())

    def _build_status(self):
        self.status = ttk.Label(self.root, anchor="w", padding=(6, 2))
        self.status.pack(fill="x")

    # ------------------------------------------------------------- drawing
    def world_to_screen(self, x, y):
        return x * self.scale + self.ox, y * self.scale + self.oy

    def screen_to_world(self, x, y):
        return ((x - self.ox) / self.scale, (y - self.oy) / self.scale)

    def fit_view(self):
        all_pts = []
        for ch in self.chains:
            all_pts.extend(ch["pts"])
        if self.cur_pts:
            all_pts.extend(self.cur_pts)
        if not all_pts:
            self.scale = 1.0
            self.ox = 0.0
            self.oy = 0.0
            self.redraw()
            return

        min_x = min(p[0] for p in all_pts)
        max_x = max(p[0] for p in all_pts)
        min_y = min(p[1] for p in all_pts)
        max_y = max(p[1] for p in all_pts)

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            w, h = 960, 700

        margin = 40
        pw = max_x - min_x
        ph = max_y - min_y
        if pw < 1e-5:
            pw = 1.0
        if ph < 1e-5:
            ph = 1.0

        scale_x = (w - 2 * margin) / pw
        scale_y = (h - 2 * margin) / ph
        self.scale = max(0.05, min(100.0, min(scale_x, scale_y)))

        cx_world = (min_x + max_x) / 2.0
        cy_world = (min_y + max_y) / 2.0
        self.ox = (w / 2.0) - cx_world * self.scale
        self.oy = (h / 2.0) - cy_world * self.scale

        self.redraw()

    def reset_view(self):
        self.scale = 1.0
        self.ox = 0.0
        self.oy = 0.0
        self.redraw()

    def redraw(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w, h = 960, 700
        if self.snap_enabled.get():
            wx0, wy0 = self.screen_to_world(0, 0)
            wx1, wy1 = self.screen_to_world(w, h)
            gx0 = int(wx0 / SNAP) * SNAP
            gx1 = int(wx1 / SNAP) * SNAP
            for gx in range(gx0, gx1 + 1, SNAP):
                sx, _ = self.world_to_screen(gx, 0)
                c.create_line(sx, 0, sx, h, fill=GRID)
            gy0 = int(wy0 / SNAP) * SNAP
            gy1 = int(wy1 / SNAP) * SNAP
            for gy in range(gy0, gy1 + 1, SNAP):
                _, sy = self.world_to_screen(0, gy)
                c.create_line(0, sy, w, sy, fill=GRID)

        for i, ch in enumerate(self.chains):
            self._draw_chain(ch["pts"], ch["type"], i, final=True)
        if self.cur_pts:
            self._draw_chain(self.cur_pts, self.cur_type, len(self.chains),
                             final=False)
        self._update_status()

    def _draw_chain(self, pts, ctype, idx, final):
        c = self.canvas
        screen = [self.world_to_screen(x, y) for x, y in pts]
        if len(screen) >= 2:
            coords = [coord for p in screen for coord in p]
            if final or (len(pts) >= 3 and self.is_closed(pts)):
                color = EDGE_OUT if ctype == "out" else EDGE_IN
                fill = FILL_OUT if ctype == "out" else ""
                if len(pts) >= 3:
                    c.create_polygon(coords, fill=fill,
                                     outline=color, width=2)
                else:
                    c.create_line(coords, fill=color, width=2)
            else:
                color = EDGE_OUT if ctype == "out" else EDGE_IN
                c.create_line(coords, fill=color, width=2, dash=(4, 2))
        r = max(2, min(6, 3 * self.scale))
        fsize = max(6, min(20, int(8 * self.scale)))
        for i, (x, y) in enumerate(screen):
            tag = "v{}_{}".format(idx, i)
            c.create_oval(x - r, y - r, x + r, y + r, fill=VERTEX,
                          outline="", tags=tag)
            c.create_text(x + 5, y - 5, text=str(i + 1), fill=TEXT,
                          font=("Helvetica", fsize), anchor="sw", tags=tag)

    def is_closed(self, pts):
        if len(pts) < 3:
            return False
        tol = CLOSE_R / self.scale
        dx = pts[-1][0] - pts[0][0]
        dy = pts[-1][1] - pts[0][1]
        return (dx * dx + dy * dy) <= (tol * tol)

    def _update_status(self):
        n = len(self.chains)
        cur = self.cur_type.upper()
        snap = "on" if self.snap_enabled.get() else "off"
        name = self.file_path or "unnamed"
        msg = ("Drawing: %s chain, %d points | committed chains: %d "
               "(%d outer, %d holes) | snap: %s | zoom: %.2fx | file: %s | "
               "Click=add/drag, Shift/Dbl-click=insert on edge, Right-click=delete vertex, Esc=close chain, "
               "Ctrl+F=Fit View") % (
            cur, len(self.cur_pts), n,
            sum(1 for ch in self.chains if ch["type"] == "out"),
            sum(1 for ch in self.chains if ch["type"] == "in"),
            snap, self.scale, name)
        self.status.config(text=msg)

    # --------------------------------------------------------------- input
    def _canvas_coords(self, e):
        if e.widget is self.canvas:
            return e.x, e.y
        return (e.x_root - self.canvas.winfo_rootx(),
                e.y_root - self.canvas.winfo_rooty())

    def _hit_vertex(self, x, y):
        chains = list(self.chains)
        if self.cur_pts:
            chains = chains + [{"type": self.cur_type, "pts": self.cur_pts}]
        tol = HIT_R / self.scale
        for ci, ch in enumerate(chains):
            for pi, p in enumerate(ch["pts"]):
                dx, dy = p[0] - x, p[1] - y
                if (dx * dx + dy * dy) <= tol * tol:
                    return (ci, pi)
        return None

    def _hit_edge(self, x, y):
        chains = list(self.chains)
        if self.cur_pts:
            chains = chains + [{"type": self.cur_type, "pts": self.cur_pts}]
        tol = (HIT_R + 4) / self.scale
        best_dist = tol * tol
        best_hit = None

        for ci, ch in enumerate(chains):
            pts = ch["pts"]
            n = len(pts)
            if n < 2:
                continue
            is_closed = (ci < len(self.chains)) or self.is_closed(pts)
            count = n if is_closed else n - 1
            for i in range(count):
                p1 = pts[i]
                p2 = pts[(i + 1) % n]
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                l2 = dx * dx + dy * dy
                if l2 == 0:
                    px, py = p1[0], p1[1]
                else:
                    t = max(0.0, min(1.0, ((x - p1[0]) * dx + (y - p1[1]) * dy) / l2))
                    px = p1[0] + t * dx
                    py = p1[1] + t * dy
                d2 = (x - px) ** 2 + (y - py) ** 2
                if d2 < best_dist:
                    best_dist = d2
                    best_hit = (ci, i + 1, (px, py))
        return best_hit

    def on_double_click(self, e):
        wx, wy = self.screen_to_world(e.x, e.y)
        edge_hit = self._hit_edge(wx, wy)
        if edge_hit is not None:
            ci, insert_idx, (proj_x, proj_y) = edge_hit
            px, py = self._snap(proj_x), self._snap(proj_y)
            if ci < len(self.chains):
                self.chains[ci]["pts"].insert(insert_idx, (px, py))
                self.drag = (ci, insert_idx)
            else:
                self.cur_pts.insert(insert_idx, (px, py))
                self.drag = (len(self.chains), insert_idx)
            self.redraw()

    def on_left_down(self, e):
        wx, wy = self.screen_to_world(e.x, e.y)
        x, y = self._snap(wx), self._snap(wy)

        # Check vertex hit for drag
        hit = self._hit_vertex(x, y)
        if hit is not None:
            self.drag = hit
            return

        # Check Shift+Click for edge vertex insertion
        if e.state & 0x0001:  # Shift key held
            edge_hit = self._hit_edge(wx, wy)
            if edge_hit is not None:
                ci, insert_idx, (proj_x, proj_y) = edge_hit
                ipx, ipy = self._snap(proj_x), self._snap(proj_y)
                if ci < len(self.chains):
                    self.chains[ci]["pts"].insert(insert_idx, (ipx, ipy))
                    self.drag = (ci, insert_idx)
                else:
                    self.cur_pts.insert(insert_idx, (ipx, ipy))
                    self.drag = (len(self.chains), insert_idx)
                self.redraw()
                return

        # Regular click: add vertex to current chain
        if self.is_closed(self.cur_pts + [(x, y)]):
            self.close_chain(force_after_close=True)
            self.cur_pts.append((self._snap(wx), self._snap(wy)))
        else:
            self.cur_pts.append((x, y))
        self.redraw()

    def on_drag(self, e):
        if self.drag is None:
            return
        ci, pi = self.drag
        wx, wy = self.screen_to_world(e.x, e.y)
        x, y = self._snap(wx), self._snap(wy)
        if ci < len(self.chains):
            self.chains[ci]["pts"][pi] = (x, y)
        else:
            self.cur_pts[pi] = (x, y)
        self.redraw()

    def on_left_up(self, e):
        self.drag = None

    def on_zoom(self, e):
        x, y = self._canvas_coords(e)
        if hasattr(e, "delta") and e.delta:
            factor = 1.1 if e.delta > 0 else 1.0 / 1.1
        else:
            factor = 1.1 if e.num == 4 else 1.0 / 1.1
        self.zoom(factor, x, y)

    def zoom(self, factor, cx=None, cy=None):
        if cx is None or cy is None:
            cx = self.canvas.winfo_width() / 2.0
            cy = self.canvas.winfo_height() / 2.0
        wx, wy = self.screen_to_world(cx, cy)
        self.scale = min(100.0, max(0.05, self.scale * factor))
        self.ox = cx - wx * self.scale
        self.oy = cy - wy * self.scale
        self.redraw()

    def on_pan_start(self, e):
        self.pan_start = (e.x, e.y, self.ox, self.oy)

    def on_pan(self, e):
        if self.pan_start is None:
            return
        sx, sy, ox, oy = self.pan_start
        self.ox = ox + (e.x - sx)
        self.oy = oy + (e.y - sy)
        self.redraw()

    def on_pan_end(self, e):
        self.pan_start = None

    def on_right_click(self, e):
        wx, wy = self.screen_to_world(e.x, e.y)
        x, y = self._snap(wx), self._snap(wy)
        hit = self._hit_vertex(x, y)
        if hit is None:
            return
        ci, pi = hit
        if ci < len(self.chains):
            ch = self.chains[ci]
            if len(ch["pts"]) <= 3:
                messagebox.showinfo(
                    "ACD2D editor",
                    "Cannot delete: chain would have fewer than 3 vertices. "
                    "Delete the whole chain instead.")
                return
            del ch["pts"][pi]
        else:
            if not self.cur_pts:
                return
            del self.cur_pts[pi]
        self.redraw()

    def _snap(self, v):
        if self.snap_enabled.get():
            return int(round(v / float(SNAP)) * SNAP)
        return int(round(v))

    # -------------------------------------------------------------- editing
    def sync_type(self):
        if self.chains:
            self.type_var.set("in")
        self.cur_type = self.type_var.get()
        self.redraw()

    def close_chain(self, force_after_close=False):
        if len(self.cur_pts) >= 3:
            if self.is_closed(self.cur_pts):
                self.cur_pts = self.cur_pts[:-1]  # drop duplicate closing pt
            pts = normalize_orientation(self.cur_pts, self.cur_type)
            self.chains.append({"type": self.cur_type, "pts": pts})
            self.cur_pts = []
            if self.chains:
                self.cur_type = "in"
                self.type_var.set("in")
        else:
            if len(self.cur_pts) < 3 and not force_after_close:
                messagebox.showinfo(
                    "ACD2D editor",
                    "A chain needs at least 3 vertices before it can be "
                    "closed.")
        if force_after_close:
            self.cur_pts = []
        self.redraw()

    def undo_point(self):
        if self.cur_pts:
            self.cur_pts.pop()
        self.redraw()

    def delete_last_chain(self):
        if self.chains:
            self.chains.pop()
            if not self.chains:
                self.cur_type = "out"
                self.type_var.set("out")
        else:
            self.cur_pts = []
        self.redraw()

    def clear_all(self):
        if messagebox.askyesno("ACD2D editor", "Clear everything?"):
            self.chains = []
            self.cur_pts = []
            self.cur_type = "out"
            self.type_var.set("out")
            self.file_path = None
            self.redraw()

    def new_file(self):
        self.clear_all()

    # ---------------------------------------------------------- file I/O
    def _all_chains(self):
        chains = [{"type": ch["type"], "pts": normalize_orientation(ch["pts"], ch["type"])}
                  for ch in self.chains]
        if self.cur_pts and len(self.cur_pts) >= 3:
            chains.append({"type": self.cur_type, "pts": normalize_orientation(self.cur_pts, self.cur_type)})
        elif self.cur_pts:
            chains.append({"type": self.cur_type, "pts": list(self.cur_pts)})
        return chains

    def save_dialog(self):
        chains = self._all_chains()
        if not chains:
            messagebox.showwarning("ACD2D editor",
                                   "Nothing to save. Draw a polygon first.")
            return
        if chains[0]["type"] != "out":
            messagebox.showerror("ACD2D editor",
                                 "The first chain must be the outer boundary.")
            return
        for ch in chains:
            if len(ch["pts"]) < 3:
                messagebox.showerror(
                    "ACD2D editor",
                    "A chain has fewer than 3 vertices (see vertex numbers). "
                    "Finish it with Esc or delete it.")
                return
        if self.file_path is None:
            self.save_as_dialog()
            return
        self.write_file(self.file_path, chains)

    def save_as_dialog(self):
        chains = self._all_chains()
        if not chains:
            messagebox.showwarning("ACD2D editor",
                                   "Nothing to save. Draw a polygon first.")
            return
        if chains[0]["type"] != "out":
            messagebox.showerror("ACD2D editor",
                                 "The first chain must be the outer boundary.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".poly",
            filetypes=[("Polygon file", "*.poly"), ("All files", "*")])
        if not path:
            return
        self.file_path = path
        self.write_file(path, chains)

    def write_file(self, path, chains):
        with open(path, "w") as f:
            f.write("# acd2d polygon editor output\n")
            f.write("%d\n" % len(chains))
            for ch in chains:
                pts = normalize_orientation(ch["pts"], ch["type"])
                f.write("%d %s\n" % (len(pts), ch["type"]))
                for x, y in pts:
                    f.write("%s %s\n" % (fmt(x), fmt(y)))
                f.write("%s\n" % " ".join(
                    str(i + 1) for i in range(len(pts))))
        self.file_path = path
        self.chains = [{"type": ch["type"], "pts": normalize_orientation(ch["pts"], ch["type"])}
                       for ch in self.chains]
        if self.root and self.root.winfo_exists():
            self.redraw()
            messagebox.showinfo("ACD2D editor",
                                "Saved %d chain(s) to:\n%s" % (len(chains), path))

    def load_file(self, path):
        try:
            chains = self.read_file(path)
        except Exception as ex:
            messagebox.showerror("ACD2D editor",
                                 "Failed to parse %s:\n%s" % (path, ex))
            return
        self.chains = [{"type": ch["type"], "pts": normalize_orientation(ch["pts"], ch["type"])}
                       for ch in chains]
        self.cur_pts = []
        self.cur_type = "in" if chains else "out"
        self.type_var.set(self.cur_type)
        self.file_path = path
        self.fit_view()

    def open_dialog(self):
        path = filedialog.askopenfilename(
            filetypes=[("Polygon file", "*.poly"), ("All files", "*")])
        if not path:
            return
        self.load_file(path)

    @staticmethod
    def read_file(path):
        num_pattern = r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?'
        word_pattern = r'[a-zA-Z_]+'
        pattern = f'{word_pattern}|{num_pattern}'

        def tokens():
            with open(path) as f:
                for line in f:
                    line = line.split("#")[0]
                    for tok in re.findall(pattern, line):
                        yield tok

        toks = tokens()
        try:
            nchains = int(next(toks))
        except StopIteration:
            raise ValueError("empty file")

        chains = []
        for _ in range(nchains):
            vsize = int(next(toks))
            ctype = next(toks).lower()
            if "out" in ctype:
                ctype = "out"
            else:
                ctype = "in"
            pts = [(float(next(toks)), float(next(toks)))
                   for _ in range(vsize)]
            order = [int(next(toks)) - 1 for _ in range(vsize)]
            ordered = [pts[i] for i in order]
            chains.append({"type": ctype, "pts": ordered})
        return chains

    # ---------------------------------------------------------------- misc
    def show_usage(self):
        messagebox.showinfo(
            "ACD2D .poly Editor - Usage",
            "LEFT CLICK         : Add a vertex / Drag an existing vertex\n"
            "DOUBLE-CLICK / SHIFT+CLICK : Insert vertex on an existing edge\n"
            "RIGHT CLICK        : Delete vertex under cursor\n"
            "CTRL+SCROLL        : Zoom in / out (anchored at cursor)\n"
            "MIDDLE-DRAG        : Pan the view\n"
            "CTRL+F             : Fit polygon view to canvas\n\n"
            "ESC                : Close current chain, start a new one\n"
            "CTRL+Z / BACKSPACE : Undo last point\n"
            "DEL                : Delete last committed chain\n\n"
            "The first chain must be OUTER (polygon boundary).\n"
            "Every chain after it is a HOLE. Each chain needs >= 3 points.\n\n"
            "LOAD / SAVE reads and writes .poly files directly.")


def main():
    root = tk.Tk()
    file_to_load = sys.argv[1] if len(sys.argv) > 1 else None
    PolyEditor(root, file_to_load=file_to_load)
    root.mainloop()


if __name__ == "__main__":
    sys.exit(main())
