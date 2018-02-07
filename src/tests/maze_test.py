# Test maze generation
print("Generating maze...")
maze = Maze.Maze()
print("Inverting maze left to right")
maze.invert()
print("Getting endpoints")
start, goal = maze.getendpoints()
print("Getting player")
player = Maze.Player()
print("Initializing display...")
pygame.init()
display_surf = pygame.display.set_mode((800,600))
pygame.display.set_caption('Pygame pythonspot.com example')
maze.draw(display_surf)
pygame.display.update()

# Test maze solving
print("Solving maze...")
came_from = a_star(maze)
path = reconstruct_path(came_from, start, goal)

# Display maze solution
for position in path:
    pixels_x = (position[0] * 50) + math.floor(abs((50 - 20) * 0.5))
    #print pixels_x
    pixels_y = (position[1] * 50) + math.floor(abs((50 - 20) * 0.5))
    player.goTo(pixels_x, pixels_y)
    player.draw(display_surf)
pygame.display.update()

# Leave window up to let user see result
running = True
while running:
    pygame.event.pump()
    keys = pygame.key.get_pressed()
    if (keys[K_ESCAPE]):
        running = False

# Exit test
pygame.quit()
