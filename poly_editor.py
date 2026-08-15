#!/usr/bin/env python3
"""
ACD2D .poly polygon editor.

Draw polygons by clicking on the canvas, add holes, then save as a .poly
file that acd2d can read directly.

.poly format written (matches acd2d's cd_polygon/cd_poly reader):
    <num_chains>
    <num_verts> out
    <x> <y>
    ...
    1 2 3 ... <num_verts>
    <num_verts> in
    <x> <y>
    ...
    1 2 3 ... <num_verts>
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import sys

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


def tokens_from_file(path):
    with open(path) as f:
        for line in f:
            line = line.split("#")[0]
            for tok in line.split():
                yield tok


class PolyEditor:
    def __init__(self, root):
        self.root = root
        root.title("ACD2D .poly Editor")
        root.geometry("920x680")

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
        root.bind("<Control-z>", lambda e: self.undo_point())
        root.bind("<Escape>", lambda e: self.close_chain())
        root.bind("<Delete>", lambda e: self.delete_last_chain())
        root.bind("<BackSpace>", lambda e: self.undo_point())

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
            side="left", padx=8)
        ttk.Button(bar, text="Undo Point", command=self.undo_point).pack(
            side="left")
        ttk.Button(bar, text="Delete Last Chain",
                   command=self.delete_last_chain).pack(side="left")
        ttk.Checkbutton(bar, text="Snap", variable=self.snap_enabled,
                        command=self.redraw).pack(side="left", padx=8)
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

    def _build_status(self):
        self.status = ttk.Label(self.root, anchor="w", padding=(6, 2))
        self.status.pack(fill="x")

    # ------------------------------------------------------------- drawing
    def world_to_screen(self, x, y):
        return x * self.scale + self.ox, y * self.scale + self.oy

    def screen_to_world(self, x, y):
        return ((x - self.ox) / self.scale, (y - self.oy) / self.scale)

    def redraw(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w <= 1:
            w, h = 920, 680
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
               "(%d outer, %d holes) | snap: %s | zoom: %dx | file: %s | "
               "Click=add/drag, Right-click=delete vertex, Esc=close chain, "
               "Ctrl+scroll=zoom, Middle-drag=pan") % (
            cur, len(self.cur_pts), n,
            sum(1 for ch in self.chains if ch["type"] == "out"),
            sum(1 for ch in self.chains if ch["type"] == "in"),
            snap, int(round(self.scale * 100)) / 100.0, name)
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

    def on_left_down(self, e):
        wx, wy = self.screen_to_world(e.x, e.y)
        x, y = self._snap(wx), self._snap(wy)
        hit = self._hit_vertex(x, y)
        if hit is not None:
            self.drag = hit
            return
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

    def zoom(self, factor, cx, cy):
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
            return
        self.cur_type = self.type_var.get()
        self.redraw()

    def close_chain(self, force_after_close=False):
        if len(self.cur_pts) >= 3:
            if self.is_closed(self.cur_pts):
                self.cur_pts = self.cur_pts[:-1]  # drop duplicate closing pt
            self.chains.append({"type": self.cur_type, "pts": self.cur_pts})
            self.cur_pts = []
            if self.chains:
                self.cur_type = "in"
                self.type_var.set("in")
        else:
            if len(self.cur_pts) < 3:
                messagebox.showinfo(
                    "ACD2D editor",
                    "A chain needs at least 3 vertices before it can be "
                    "closed.")
        if force_after_close:
            # user clicked to close; start a new empty chain right away
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
        chains = [{"type": ch["type"], "pts": list(ch["pts"])}
                  for ch in self.chains]
        if self.cur_pts:
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
                pts = ch["pts"]
                f.write("%d %s\n" % (len(pts), ch["type"]))
                for x, y in pts:
                    f.write("%s %s\n" % (fmt(x), fmt(y)))
                f.write("%s\n" % " ".join(
                    str(i + 1) for i in range(len(pts))))
        self.file_path = path
        if self.root.winfo_exists():
            self.redraw()
            messagebox.showinfo("ACD2D editor",
                                "Saved %d chain(s) to:\n%s" % (len(chains), path))

    def open_dialog(self):
        path = filedialog.askopenfilename(
            filetypes=[("Polygon file", "*.poly"), ("All files", "*")])
        if not path:
            return
        try:
            chains = self.read_file(path)
        except Exception as ex:
            messagebox.showerror("ACD2D editor",
                                 "Failed to parse %s:\n%s" % (path, ex))
            return
        self.chains = chains
        self.cur_pts = []
        self.cur_type = "out"
        self.type_var.set("out")
        self.file_path = path
        self.redraw()
        messagebox.showinfo("ACD2D editor",
                            "Loaded %d chain(s)." % len(chains))

    @staticmethod
    def read_file(path):
        toks = tokens_from_file(path)
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
            "LEFT CLICK   : add a vertex / drag an existing vertex\n"
            "RIGHT CLICK  : delete a vertex under the cursor\n"
            "CTRL+SCROLL  : zoom in / out (anchored at the cursor)\n"
            "MIDDLE-DRAG  : pan the view\n"
            "Click near the first vertex of the current chain to close it.\n\n"
            "ESC          : close current chain, start a new one\n"
            "CTRL+Z       : undo last point\n"
            "DEL          : delete last committed chain\n\n"
            "The first chain must be OUTER (the polygon boundary).\n"
            "Every chain after it is a HOLE. Each chain needs >= 3 points.\n\n"
            "SAVE writes a .poly file that acd2d_gui reads directly.")


def main():
    root = tk.Tk()
    PolyEditor(root)
    root.mainloop()


if __name__ == "__main__":
    sys.exit(main())
