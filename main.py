

import json, time , pygame, sys, win32api
import numpy as np
import ui as ui
from nonogram import Nonogram as nono
from kakurasu import Kakurasu as kaku
from pygame.locals import *
from constants_main import *
from optionbox import OptionBox

font = pygame.font.Font('freesansbold.ttf', 28)
tfont = pygame.font.Font('freesansbold.ttf', 35)
font_c = pygame.font.Font('freesansbold.ttf', 25)
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
mainClock = pygame.time.Clock()
pygame.display.set_caption('Assignment')

#var
list1 = OptionBox(
    (screen.get_width() // 2) - SQUARE_SIZE * (4.2/2),
    (screen.get_height() // 2) - (SQUARE_SIZE * (1.2/2))
    ,  SQUARE_SIZE * 4.2,  SQUARE_SIZE  * 1.2 , GREY, DARK_GREY, font_c, 
    ["4x4", "5x5", "6x6", "7x7"])

def main(): 
    
    run = True
    setting = []
    while run:
        mainClock.tick(60)
        WIN.fill(GREY)
        title = tfont.render('Click me', True, WHITE, None)
        text_rect = title.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 - 100))
        WIN.blit(title, text_rect)
       
        event_list = pygame.event.get()
        
        for event in event_list:
            if event.type == pygame.QUIT:
                run = False
        N = list1.update(event_list)
        if N >= 0:
            if N == 0: 
                f_n = open("./introAI/nonogram/data/4x4.json")
                f_k= open("./introAI/kakurasu/data/4x4.json")
            elif N == 1:  
                f_n = open("./nonogram/data/5x5.json")
                f_k = open("./kakurasu/data/5x5.json")
            elif N == 2: 
                f_n = open("./nonogram/data/6x6.json")
                f_k = open("./kakurasu/data/6x6.json")    
            elif N == 3: 
                f_n = open("./introAI/nonogram/data/7x7.json")
                f_k = open("./introAI/kakurasu/data/7x7.json")
            data_n = json.load(f_n)
            board_n = np.zeros((data_n["dimention"], data_n["dimention"]))
            col_nono = data_n["col"]
            row_nono = data_n["row"]
            problem_nono = nono.NonogramProblem(board_n, row_nono, col_nono)
            solving_nono = nono.NonogramSolver(problem_nono)
            setting.append(data_n)
            data_k = json.load(f_k)
            board_k = np.zeros((data_k["dimention"], data_k["dimention"]))
            col_kaku = data_k["col"]
            row_kaku = data_k["row"]
            problem_kaku = kaku.KakurasuProblem(
                board_k, row_kaku, col_kaku, data_k["dimention"])
            solving_kaku = kaku.KakurasuSolver(problem_kaku)
            setting.append(data_k)
            data_n = None
            data_k = None
            ui.draw(solving_nono, solving_kaku, setting)
        setting = []
        list1.draw(WIN)
        pygame.display.flip()
        
if __name__ == "__main__":
    pygame.init()
    main()
    pygame.quit()