from macros import *
import numpy as np
from gworld import *
from tkinter import *


class Visualize:
    def __init__(self, world_data):
        self.frame = Tk()
        self.canvas = Canvas(self.frame, width=FRAME_WIDTH, height=FRAME_HEIGHT)
        self.canvas.configure(bg='white')
        self.canvas.grid()
        self.world = world_data
        world_data.visualize = self
        self.cell_h, self.cell_w = self.get_cell_size()
        self.agent_h, self.agent_w = self.get_agent_size(1)
        self.vis_cells = np.zeros_like(self.world.cells, dtype = int)
        self.aindx_obj = dict()
        self.aindx_obj_text = dict()

    def root(self):
        return self.frame

    def create_button(self, func):
        button = Button(self.frame, text="TestStop", command=func)
        button.pack(side="top", anchor="nw")

    def draw_world(self):
        nrows, ncols = self.world.get_size()
        for row in range(nrows):
            for col in range(ncols):
                x = FRAME_MARGIN + self.cell_w * col
                y = FRAME_MARGIN + self.cell_h * row
                self.canvas.create_text(x+15, y+5, text=str(row)+", "+str(col), fill="black", font=("Arial", 7));
                self.vis_cells[row][col] = self.canvas.create_rectangle(x, y, FRAME_MARGIN + self.cell_w * (col+1), FRAME_MARGIN + self.cell_h * (row+1) )
                if self.world.cells[row][col] == IS_ROCK:
                    self.canvas.itemconfig(self.vis_cells[row][col], fill='gray', width=2)

    def draw_path(self, path, num):
        for point in path[:-1]:
            y1, x1, y2, x2 = self.get_pos_in_cell(point[0], point[1])
            self.canvas.create_oval(x1+10, y1+10, x2-10, y2-10, fill=COLORS[num], width=0)

    def clear_cell(self, point):
        y1, x1, y2, x2 = self.get_pos_in_cell(point[0], point[1])
        self.canvas.create_oval(x1 + 10, y1 + 10, x2 - 10, y2 - 10, fill='white', width=0)

    def get_pos_in_cell(self, crow, ccol):
        agent_h = self.agent_h
        agent_w = self.agent_w
        agent_y1 = FRAME_MARGIN + (crow * self.cell_h) + CELL_MARGIN
        agent_y2 = agent_y1 + agent_h
        agent_x1 = FRAME_MARGIN + (ccol * self.cell_w) + CELL_MARGIN
        agent_x2 = agent_x1 + agent_w
        return agent_y1, agent_x1, agent_y2, agent_x2

    def draw_agents(self):
        for crow in range(self.world.h):
            for ccol in range(self.world.w):
                cell = self.world.cells[crow][ccol]
                if cell != UNOCCUPIED and not self.world.is_blocked(crow, ccol):
                    y1, x1, y2, x2 = self.get_pos_in_cell(crow, ccol)
                    self.aindx_obj[cell] = self.canvas.create_oval(x1, y1, x2, y2, fill=COLORS[cell])
                    self.aindx_obj_text[cell] = self.canvas.create_text(x1 + (x2-x1)/2, y1 + (y2-y1)/2, text=str(cell))
                    gy, gx = self.world.aindx_goal[cell]
                    goal_cell = self.vis_cells[gy][gx]
                    self.canvas.itemconfig(goal_cell, outline=COLORS[cell], width=4)

    def update_agent_vis(self, aindx):
        cy, cx = self.world.aindx_cpos[aindx]
        y1, x1, y2, x2 = self.get_pos_in_cell(cy, cx)
        self.canvas.coords(self.aindx_obj[aindx], x1, y1, x2, y2)
        self.canvas.coords(self.aindx_obj_text[aindx], x1 + (x2-x1)/2, y1 + (y2-y1)/2)

    def get_cell_size(self):
        avail_h = FRAME_HEIGHT - 2 * FRAME_MARGIN
        avail_w = FRAME_WIDTH - 2 * FRAME_MARGIN
        nrows, ncols = self.world.get_size()
        cell_h = avail_h / nrows
        cell_w = avail_w / ncols
        return cell_h, cell_w

    def get_agent_size(self, nagents):
        agent_h = self.cell_h - 2 * CELL_MARGIN
        agent_w = self.cell_w - 2 * CELL_MARGIN
        return agent_h, agent_w

    def do_loop(self):
        self.frame.mainloop()

    def do_pack(self):
        self.canvas.pack()
