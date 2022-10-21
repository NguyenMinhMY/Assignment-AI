import pygame
from .constants_kaku import *
from .check import *
pygame.font.init()

INDEX = []
class Board_kaku():
    def __init__(self,data_k):
        self.N = data_k["dimention"]
        self.board = []
        self.selected_piece = None
        self.row_req = data_k["row"]
        self.col_req = data_k["col"]
        self.ROWS = self.N + 2
        self.COLS =  self.N + 2
        self.SQUARE_SIZE = 768//self.COLS
        self.BORDER = round(self.SQUARE_SIZE / 21)
        self.row_test = np.zeros((self.N,self.N))
        self.create_board()
        for i in range(1,self.N+1):
          INDEX.append(str(i))

    def draw_square(self, win):
        win.fill (BLACK)
        for row in range(self.ROWS):
            for col in range(self.COLS):
                pygame.draw.rect(win, GREY, (row * self.SQUARE_SIZE, col * self.SQUARE_SIZE, self.SQUARE_SIZE - self.BORDER, self.SQUARE_SIZE - self.BORDER))
        for row in range(self.ROWS):
            for col in range(1):
                pygame.draw.rect(win, GREY, (row * self.SQUARE_SIZE, col * self.SQUARE_SIZE, self.SQUARE_SIZE, self.SQUARE_SIZE - self.BORDER ))
        for col in range(self.COLS):
            for row in range(1):
                pygame.draw.rect(win, GREY, (row * self.SQUARE_SIZE, col * self.SQUARE_SIZE, self.SQUARE_SIZE - self.BORDER, self.SQUARE_SIZE ))
        for row in range(self.ROWS):
            for col in range(self.COLS-1, self.COLS):
                pygame.draw.rect(win, GREY, (row * self.SQUARE_SIZE, col * self.SQUARE_SIZE, self.SQUARE_SIZE, self.SQUARE_SIZE))
        for col in range(self.COLS):
            for row in range(self.ROWS-1, self.ROWS):
                pygame.draw.rect(win, GREY, (row * self.SQUARE_SIZE, col * self.SQUARE_SIZE, self.SQUARE_SIZE, self.SQUARE_SIZE))
        
    def create_board(self):
        for row in range(self.ROWS-1):
            self.board.append([])
            for col in range(self.COLS-1):
                temp = self.row_test[row-1][col-1]
                # print(temp)
                if (temp == 1):
                    self.board[row].append(Check(row, col, GREY_GREEN, self.SQUARE_SIZE, self.BORDER))
                else:
                    self.board[row].append(Check(row,col, GREY_RED, self.SQUARE_SIZE, self.BORDER))
    def draw(self, win):
        self.draw_square(win)
        for row in range(self.ROWS-2):
            for col in range(self.COLS-2):
                check = self.board[row][col]
                if check == 0:
                    check.draw_square(win)
                else:
                    check.draw_square(win)
       
        #add text
        font = pygame.font.Font('freesansbold.ttf', 24 - self.N)
        text = font.render('Geeks', True, WHITE, BLACK)
        new_index = list()
        for row in INDEX:
            new_index += ['   '.join((str(e) for e in row))] 
        for col in INDEX:
            new_index += ['   '.join((str(e) for e in col))]
        for row in range(1,self.ROWS-1):
            col = 0
            text = font.render(new_index[row-1], True, LIGHT_GREY, None)
            text_rect = text.get_rect(center=( (row * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2), (col * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2)))
            win.blit(text, text_rect)
        for col in range(1,self.COLS-1):
            row = 0
            text = font.render(new_index[col-1], True, LIGHT_GREY, None)
            text_rect = text.get_rect(center=( (row * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2), (col * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2)))
            win.blit(text, text_rect)
    
        new_row_req,new_col_req = list(), list()
        for row in self.row_req:
            new_row_req += [' '.join((str(e) for e in row))] 
        for col in self.col_req:
            new_col_req += [' '.join((str(e) for e in col))]
        for row in range(1,self.ROWS-1):
            col = self.COLS-1
            text = font.render(new_row_req[row-1], True, BLACK, None)
            text_rect = text.get_rect(center=( (row * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2), (col * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2)))
            win.blit(text, text_rect)
        for col in range(1,self.COLS-1):
            row = self.ROWS-1
            text = font.render(new_col_req[col-1], True, BLACK, None)
            text_rect = text.get_rect(center=( (row * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2), (col * self.SQUARE_SIZE) + (self.SQUARE_SIZE//2)))
            win.blit(text, text_rect)
    
    def update(self, win, board_test):
        self.board = []
        for row in range(self.ROWS-2):
            self.board.append([])
            for col in range(self.COLS-2):
                temp = board_test[row][col]
                if (temp == 1):
                    self.board[row].append(Check(row, col, GREY_RED, self.SQUARE_SIZE, self.BORDER))
                else:
                    self.board[row].append(Check(row,col, GREY_GREEN, self.SQUARE_SIZE, self.BORDER))
    def __del__(self):
        self.N = 0
        self.board = []
        self.selected_piece = None
        self.row_req = []
        self.col_req = []
        self.ROWS = 0
        self.COLS =  0
        self.SQUARE_SIZE = 0
        self.BORDER = 0