f = open("tests/input8.txt", "r")

lines = f.readlines()

gameType = 0
color = 1
time = 100

board = []

for idx, i in enumerate(lines):
    i = i.strip()
    if idx == 0:
        gameType = i
    elif idx == 1:
        if i == "WHITE":
            color = 0
    elif idx > 2:
        board.append(list(i))

# Update Move

f = open("output.txt", "r")

lines = f.readlines()

start = 0
end = 0

for idx, i in enumerate(lines):
    i = i.strip()
    moves = i.split(' ')
    if idx == 0:
        start = map(int,moves[1].split(','))
    if idx == len(lines) - 1:
        end = map(int,moves[-1].split(','))

board[end[1]][end[0]] = board[start[1]][start[0]]
board[start[1]][start[0]] = '.'

f= open("tests/input8.txt","w")

f.write(gameType + '\n')

if color == 0:
    f.write("BLACK\n")
else:
    f.write("WHITE\n")

f.write(str(time) + '\n')

for i in board:
    f.write("".join(i) +'\n')
