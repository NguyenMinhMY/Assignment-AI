import pygame

WIDTH, HEIGHT = 768, 768
SQUARE_SIZE = 60
# RGB COLORS
GREY =  (78,88,99)
DARK_GREY = (64,65,75)

RED = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
LIGHT_BLACK = (82, 74, 78)
BLUE = (0, 0, 255)
GREY = (125, 125, 125)

# BUTTON POSITIONS
res = (WIDTH, HEIGHT)
screen = pygame.display.set_mode(res)
menu_btn = [(screen.get_width() // 2) - SQUARE_SIZE * 2, (screen.get_height() // 2) - (SQUARE_SIZE / 2)]
