import pygame,sys
import win32api
from pygame.locals import *
from constants_main import *
from nonogram.board import Board_nono
from kakurasu.board import Board_kaku
from optionbox import OptionBox
import main as main
FPS = 60
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
mainClock = pygame.time.Clock()
pygame.init()
pygame.display.set_caption('Assignment')

font = pygame.font.Font('freesansbold.ttf', 28)
tfont = pygame.font.Font('freesansbold.ttf', 35)

def draw(nono, kaku, setting):
    a = True

    while a:
        WIN.fill(GREY)
        mx, my = pygame.mouse.get_pos()
        #draw outside button
        btn_1 = pygame.draw.rect(WIN, WHITE, ((screen.get_width() // 2) - SQUARE_SIZE * (4.2/2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2/2)), SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        btn_2 = pygame.draw.rect(WIN, WHITE, (
        (screen.get_width() // 2) - SQUARE_SIZE * (4.2 / 2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2 / 2) - 1.5*SQUARE_SIZE),
        SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        #draw inside button
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))
        #hover
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] <= my <= menu_btn[1] + SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] + 1.5*SQUARE_SIZE <= my <= menu_btn[1] + SQUARE_SIZE + 1.5*SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))
            
        #add text
        title = tfont.render('Choose Game', True, WHITE, None)
        text_rect =   title.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 - 100))
        WIN.blit(title, text_rect)
        text1 = font.render('Nonogram', True, WHITE, None)
        text_rect = text1.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2))
        WIN.blit(text1, text_rect)
 
        text2 = font.render('Kakurasu', True, WHITE, None)
        text_rect = text2.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2 + 1.5*SQUARE_SIZE))
        WIN.blit(text2, text_rect)
        click = False
        event_list = pygame.event.get()
        for event in event_list:
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    a = False
          
                    
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: 
                    click = True
        
        if btn_1.collidepoint((mx, my)):
            if click:
                nonogram(nono, setting[0])
        if btn_2.collidepoint((mx, my)):
            if click:
                kakurasu(kaku, setting[1])
        
        pygame.display.update() 
        mainClock.tick(FPS)

def nonogram(nono, data):
    running = True
    while running: 
        WIN.fill(GREY)
        mx, my = pygame.mouse.get_pos()
        #draw outside button
        btn_nono_1 = pygame.draw.rect(WIN, WHITE, ((screen.get_width() // 2) - SQUARE_SIZE * (4.2/2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2/2)), SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        btn_nono_2 = pygame.draw.rect(WIN, WHITE, (
        (screen.get_width() // 2) - SQUARE_SIZE * (4.2 / 2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2 / 2) - 1.5*SQUARE_SIZE),
        SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        #draw inside button
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))
        #hover
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] <= my <= menu_btn[1] + SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))
            
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] + 1.5*SQUARE_SIZE <= my <= menu_btn[1] + SQUARE_SIZE + 1.5*SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))
        #add text  
        title = tfont.render('Nonogram Solver', True, WHITE, None)
        text_rect =   title.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 - 100))
        WIN.blit(title, text_rect)
        text1 = font.render('Blinded', True, WHITE, None)
        text_rect = text1.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2))
        WIN.blit(text1, text_rect)
    
        text2 = font.render('Heuristic', True, WHITE, None)
        text_rect = text2.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2 + 1.5*SQUARE_SIZE))
        WIN.blit(text2, text_rect)
        
        click = False
       
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: 
                    click = True
           
            if btn_nono_1.collidepoint((mx, my)):
                if click:
                    nono_ani(nono.history_DFS, nono.TimeExe_DFS(), nono.MemoryLoss_DFS(), data)
            if btn_nono_2.collidepoint((mx, my)):
                if click:
                   nono_ani(nono.history_A_star, nono.TimeExe_A_star(), nono.MemoryLoss_A_star(), data)
        mainClock.tick(FPS) 
        pygame.display.update()    

def nono_ani(log, time, mem, data):
    board = Board_nono(data)
    index = 0
    run = True
    while run:
        board.draw_square(WIN)
        board.draw(WIN)
        

        if index < len(log):    
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        run = False
                        nonogram()
            
            board.update(WIN,log[index])
            index += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    win32api.MessageBox(0, "Real time execution: " + str(round(time,6)) + " (s)\nSteps: " + str(len(log))  + "\nMemory Loss: " + str(mem) + "\t(Bytes)", "Evaluate")
                    run = False
                    del board
           
        mainClock.tick(FPS)      
        pygame.display.update() 

def kakurasu(kaku, data):
    running = True
    while running:
        WIN.fill(GREY)
        mx, my = pygame.mouse.get_pos()
        #draw outside button
        btn_kaku_1 = pygame.draw.rect(WIN, WHITE, ((screen.get_width() // 2) - SQUARE_SIZE * (4.2/2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2/2)), SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        btn_kaku_2 = pygame.draw.rect(WIN, WHITE, (
        (screen.get_width() // 2) - SQUARE_SIZE * (4.2 / 2), (screen.get_height() // 2) - (SQUARE_SIZE * (1.2 / 2) - 1.5*SQUARE_SIZE),
        SQUARE_SIZE * 4.2, SQUARE_SIZE * 1.2))
        #draw inside button
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))
        pygame.draw.rect(WIN, GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))
        #hover
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] <= my <= menu_btn[1] + SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1], SQUARE_SIZE * 4, SQUARE_SIZE))  
        if menu_btn[0] <= mx <= menu_btn[0] + (SQUARE_SIZE * 4) and menu_btn[1] + 1.5*SQUARE_SIZE <= my <= menu_btn[1] + SQUARE_SIZE + 1.5*SQUARE_SIZE:
            pygame.draw.rect(WIN, DARK_GREY, (menu_btn[0], menu_btn[1] + 1.5*SQUARE_SIZE, SQUARE_SIZE * 4, SQUARE_SIZE))      
        #add text
        title = tfont.render('Kakurasu Solver', True, WHITE, None)
        text_rect =   title.get_rect(center=(screen.get_width() // 2, screen.get_height() // 2 - 100))
        WIN.blit(title, text_rect)
        text1 = font.render('Blinded', True, WHITE, None)
        text_rect = text1.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2))
        WIN.blit(text1, text_rect)
        text2 = font.render('Heuristic', True, WHITE, None)
        text_rect = text2.get_rect(
            center=(screen.get_width() // 2, screen.get_height() // 2 + 1.5*SQUARE_SIZE))
        WIN.blit(text2, text_rect)
        
        click = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                   running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: 
                    click = True

            if btn_kaku_1.collidepoint((mx, my)):
                if click:
                    kaku_ani(kaku.history_DFS, kaku.TimeExe_DFS(), kaku.MemoryLoss_DFS(), data)
            if btn_kaku_2.collidepoint((mx, my)):
                if click:
                   kaku_ani(kaku.history_A, kaku.TimeExe_A_star(), kaku. MemoryLoss_A_star(), data)

        pygame.display.update() 
        mainClock.tick(FPS)
        
def kaku_ani(log, time, mem, data):
    board = Board_kaku(data)
    index = 0
    run = True
    while run:
        board.draw_square(WIN)
        board.draw(WIN)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    win32api.MessageBox(0, "Real time execution: " + str(round(time,6)) + " (s)\nSteps: " + str(len(log)) + "\nMemory Loss: " + str(mem) + "\t(Bytes)", "Evaluate")
                    run = False
                    del board
        
        
        if index < len(log):    
            board.update(WIN,log[index])
            index += 1
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        kakurasu()

        mainClock.tick(FPS)
        pygame.display.update() 
