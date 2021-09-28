import pygame
import math
import queue

SIZE = 512
SQUARE_COUNT = 32
SQUARE_SIZE = SIZE // SQUARE_COUNT
pygame.font.init()

# colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
ORANGE = (255, 165, 0)
BLUE = (0, 0, 255)
GREY = (128, 128, 128)
PURPLE = (75, 0, 130)


class Interface:

    def __init__(self):
        self._running = True
        self.start = None
        self.end = None
        self.finished = False
        self.step = False
        self.animation = True
        self.size = SIZE, SIZE
        pygame.init()

        self.window = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("A* PathFinding")
        self.grid = Grid(SIZE, SIZE // SQUARE_COUNT, self.window)
        self.grid.init_grid()
        self.grid.draw_grid()
        self.mainloop()

    def mainloop(self):
        while self._running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self._running = False
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # left click - set barriers
                        while pygame.mouse.get_pressed(3)[0]:
                            pos = pygame.mouse.get_pos()
                            loc = (pos[0] // SQUARE_SIZE, pos[1] // SQUARE_SIZE)
                            square = self.grid.grid[loc[1]][loc[0]]
                            if not (square.is_start or square.is_end):
                                square.set_barrier()
                                pygame.display.update()
                            for ev in pygame.event.get():
                                if ev.type == pygame.MOUSEBUTTONUP:
                                    break
                    if event.button == 3:  # right click - set starting and ending points
                        pos = pygame.mouse.get_pos()
                        loc = (pos[0] // SQUARE_SIZE, pos[1] // SQUARE_SIZE)
                        square = self.grid.grid[loc[1]][loc[0]]
                        if not self.start:
                            square.set_start()
                            pygame.display.update()
                            self.start = square
                            square.h = Square.dist(square, self.end)
                            self.start.update_f()

                        elif not self.end:
                            square.set_end()
                            pygame.display.update()
                            self.end = square
                            square.h = Square.dist(square, self.end)
                            self.start.update_f()
                if event.type == pygame.KEYDOWN:
                    # Enter - start in "step mode", Space - start normally with animation, any other key - no animation
                    if event.key in [pygame.K_SPACE, pygame.K_RETURN]:
                        self.animation = True
                    else:
                        self.animation = False
                    if not self.finished:
                        if self.start and self.end:
                            if event.key == pygame.K_RETURN:
                                self.step = True
                            self.remove_neighbors()
                            self.run_algorithm()
                    else:  # pressing space after the algorithm has finished will reset the window
                        self.reset()
        pygame.quit()

    def remove_neighbors(self):
        # remove barrier from the neighbor list of all squares
        for i in range(SQUARE_COUNT):
            for square in self.grid.grid[i]:
                for j in range(8):
                    if square.neighbors[j]:
                        if square.neighbors[j].is_barrier:
                            square.neighbors[j] = None

    def run_algorithm(self):
        square_queue = queue.PriorityQueue()
        square_obj = QueueObject(self.start)
        square_queue.put(square_obj)  # the priority is f value > h value > arbitrary count value
        in_queue = {self.start}

        while not square_queue.empty():
            new = square_queue.get()
            if not new.valid:  # queue item is no longer valid and was replaced by an updated item
                continue
            current_square = new.get_square()
            if self.step:
                while 1:
                    event = pygame.event.wait()
                    if event.type == pygame.QUIT:
                        self._running = False
                        return
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.reset()
                            return
                        break

                current_square.color = BLUE
                current_square.draw()
                pygame.display.update()

            in_queue.remove(current_square)
            if current_square == self.end:
                prev = self.end.prev
                while prev != self.start:
                    prev.set_path()
                    if self.animation:
                        pygame.display.update()
                    prev = prev.prev
                self.finished = True
                pygame.display.update()
                return True

            if self.step:
                while 1:
                    event = pygame.event.wait()
                    if event.type == pygame.QUIT:
                        self._running = False
                        return
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.reset()
                            return
                        break

            for i in range(8):
                neighbor = current_square.neighbors[i]
                if not neighbor or neighbor.is_closed:
                    continue
                neighbor_dist = Square.dist(current_square, neighbor)
                f_updated = False
                if neighbor.g > current_square.g + neighbor_dist:
                    neighbor.g = current_square.g + neighbor_dist
                    neighbor.prev = current_square
                    f_updated = True
                neighbor.h = Square.dist(neighbor, self.end)
                neighbor.update_f()
                if neighbor not in in_queue:
                    in_queue.add(neighbor)
                    neighbor_obj = QueueObject(neighbor)
                    square_queue.put(neighbor_obj)
                    neighbor.set_open()
                    if self.animation:
                        pygame.display.update()
                else:
                    if f_updated:  # if the value of f was updated, invalidate the current queue entry and add a new one
                        neighbor.queue_item.invalidate()
                        neighbor.queue_item = None
                        neighbor_obj = QueueObject(neighbor)
                        square_queue.put(neighbor_obj)

                if self.step:
                    neighbor.draw()
            current_square.set_closed()
            if self.animation:
                pygame.display.update()
        self.finished = True
        pygame.display.update()
        return False

    def reset(self):
        self.finished = False
        self.start = None
        self.end = None
        self.step = False
        self.animation = True
        self.grid.reset_grid()
        pygame.display.update()


class Square:

    def __init__(self, row, col, size, color=WHITE, window=None):
        self.row = row
        self.col = col
        self.color = color
        self.neighbors = [None, None, None, None, None, None, None, None]  # in order starting from top going clockwise
        self.window = window
        self.size = size
        self.is_start = False
        self.is_end = False
        self.is_barrier = False
        self.is_closed = False
        self.g = math.inf
        self.h = math.inf
        self.f = math.inf
        self.prev = None
        self.font = pygame.font.SysFont("Arial", SQUARE_SIZE // 2)
        self.queue_item = None

    def get_location(self):
        return self.row, self.col

    def set_start(self):
        self.color = PURPLE
        self.draw()
        self.is_start = True
        self.is_barrier = False
        self.g = 0

    def set_end(self):
        self.color = ORANGE
        self.draw()
        self.is_end = True
        self.is_barrier = False
        self.h = 0

    def set_open(self):
        if not (self.is_start or self.is_end):
            self.color = GREEN
            self.draw()

    def set_closed(self):
        if not (self.is_start or self.is_end):
            self.color = RED
            self.draw()
            self.is_closed = True

    def set_path(self):
        if not (self.is_start or self.is_end):
            self.color = BLUE
            self.draw()

    def set_barrier(self):
        self.color = BLACK
        self.draw()
        self.is_barrier = True

    def update_f(self):
        self.f = self.g + self.h

    def draw(self):
        pygame.draw.rect(self.window, self.color, (self.col * self.size, self.row * self.size, self.size, self.size))
        text = self.font.render(str(self.f), True, BLACK)
        if self.f != math.inf:
            self.window.blit(text, (self.col * self.size, self.row * self.size))

    def add_neighbor(self, side, neighbor):
        # side is an integer that matches the side's index in the neighbors list
        self.neighbors[side] = neighbor
        neighbor.neighbors[(side - 4) % 8] = self  # update the neighbor's list as well

    @staticmethod
    def dist_test(square1, square2):
        if not square1 or not square2:
            return math.inf
        diag_dist = int(10 * math.sqrt(2) * min(abs(square1.col - square2.col), abs(square1.row - square2.row)))
        straight_dist = 10 * (max(abs(square1.col - square2.col), abs(square1.row - square2.row)) -
                        min(abs(square1.col - square2.col), abs(square1.row - square2.row)))
        return diag_dist + straight_dist

    @staticmethod
    def dist(square1, square2):
        if not square1 or not square2:
            return math.inf
        estimated_dist = math.sqrt(math.pow(square1.col - square2.col, 2) + math.pow(square1.row - square2.row, 2))
        return int(10*estimated_dist)


class Grid:

    def __init__(self, size, square_size, window):
        self.size = size
        self.square_size = square_size
        self.squares_num = size // square_size
        self.grid = [[] for i in range(self.squares_num)]  # create an empty grid
        self.window = window

    def init_grid(self):
        for i in range(self.squares_num):
            for j in range(self.squares_num):
                self.grid[i].append(Square(i, j, self.square_size, WHITE, window=self.window))
                if i > 0:
                    self.grid[i][j].add_neighbor(0, self.grid[i - 1][j])
                    if j > 0:
                        self.grid[i][j].add_neighbor(7, self.grid[i - 1][j - 1])
                    if j < self.squares_num - 1:
                        self.grid[i][j].add_neighbor(1, self.grid[i - 1][j + 1])
                if j > 0:
                    self.grid[i][j].add_neighbor(6, self.grid[i][j - 1])

    def draw_grid(self):

        # color each square in the grid
        for i in range(self.squares_num):
            for j in range(self.squares_num):
                self.grid[i][j].draw()

        # draw the lines of the grid
        for i in range(self.squares_num):
            # draw the lines of the grid
            pygame.draw.line(self.window, GREY, (0, i * self.square_size), (self.size, i * self.square_size))
        for j in range(self.squares_num):
            pygame.draw.line(self.window, GREY, (j * self.square_size, 0), (j * self.square_size, self.size))

        pygame.display.update()

    def reset_grid(self):
        self.grid = [[] for i in range(self.squares_num)]  # create an empty grid
        self.init_grid()
        self.draw_grid()


class QueueObject:
    """ instances of this class are inserted into the priority queue instead of the squares themselves, each instance
    is linked to a square and each square can be linked to up to 1 QueueObject.
    this allows to manually set the Queue comparison function (with __lt__) and to insert new entities into the
    queue when a square in the queue is updated, without having duplicate squares in the queue, by using the 'valid'
    field"""

    count = 0

    def __init__(self, square):
        self.square = square
        self.idx = QueueObject.count
        QueueObject.count += 1
        self.f = square.f
        self.h = square.h
        self.valid = True  # if it is false, the QueueObject is no longer linked to a square and should be ignored
        square.queue_item = self

    def invalidate(self):
        self.valid = False

    def get_square(self):
        return self.square

    def __lt__(self, other):
        # order - f value first, h value second and (arbitrary) index last
        if self.f < other.f:
            return True
        if self.f == other.f:
            if self.h < other.h:
                return True
            if self.h == other.h:
                if self.idx < other.idx:
                    return True
        return False




if __name__ == "__main__":
    game = Interface()
