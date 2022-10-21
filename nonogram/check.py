from .constants_nono import *

class Check:
    PADDING = 5

    def __init__(self, row, col, color, sq, br):
        self.row = row+1
        self.col = col+1
        self.color = color
        self.state = False
        self.x = 0
        self.y = 0
        self.SQUARE_SIZE = sq
        self.BORDER = br
        self.calc_pos()
       

    def calc_pos(self):
        x = 0
        y = 0
        self.x = self.SQUARE_SIZE * self.col 
        self.y = self.SQUARE_SIZE * self.row

    def checked(self):
        self.state = True

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.SQUARE_SIZE - self.BORDER, self.SQUARE_SIZE - self.BORDER), )
        
    def __repr__(self):
        return str(self.color)