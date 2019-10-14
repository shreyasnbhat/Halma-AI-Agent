#include <bits/stdc++.h>
#include <time.h>

using namespace std;

// x is col#, y is row#
#define convert1D(x, y) y * 16 + x
#define ROW(val) val / 16
#define COL(val) val % 16

template<typename T>
std::vector<T> make1D(std::size_t capacity) {
    return std::vector<T>(capacity);
}

int jumpNeighbours[8][2] = {
        {-2, -2},
        {-2, 0},
        {-2, 2},
        {0,  2},
        {2,  2},
        {2,  0},
        {2,  -2},
        {0,  -2}
};

int neighbours[8][2] = {
        {-1, -1},
        {-1, 0},
        {-1, 1},
        {0,  1},
        {1,  1},
        {1,  0},
        {1,  -1},
        {0,  -1}
};

void printBoard(vector<string> board) {
    for (auto i = 0; i < board.size(); i++) {
        for (auto j = 0; j < board[0].size(); j++) {
            cout << board[i][j] << " ";
        }
        cout << endl;
    }
}

class Move {
public:
    pair<pair<int, int>, pair<int, int>> move;

    Move() {
        this->move = make_pair(make_pair(-1, -1), make_pair(-1, -1));
    }

    Move(int x, int y, int xd, int yd) {
        pair<int, int> start = make_pair(x, y);
        pair<int, int> end = make_pair(xd, yd);
        this->move = make_pair(start, end);
    }

    Move(pair<int, int> first, pair<int, int> second) {
        this->move = make_pair(first, second);
    }

    string getRepr() {
        return "E " + to_string(move.first.first) + "," + to_string(move.first.second) + " " +
               to_string(move.second.first) + "," + to_string(move.second.second) + "\n";
    }
};

class JumpMove {
public:
    vector<Move> moves;

    void addMove(Move move) {
        moves.push_back(move);
    }

    string getRepr() {
        string repr = "";
        for (auto i = moves.begin(); i != moves.end(); i++) {
            repr.append("J " + to_string(i->move.first.first) + "," + to_string(i->move.first.second) + " " +
                        to_string(i->move.second.first) + "," + to_string(i->move.second.second) + "\n");
        }
        return repr;
    }
};

class MultiMove {
public:

    Move adjacent;
    bool isAdjacentMove;

    JumpMove jump;
    bool isJumpMove;

    MultiMove() {
        isAdjacentMove = false;
        isJumpMove = false;
    }

    void setJumpMove(JumpMove move) {
        isJumpMove = true;
        jump = move;
    }

    void setAdjacentMove(Move move) {
        isAdjacentMove = true;
        adjacent = move;
    }

    pair<int, int> getStartFromMultiMove() {
        int x;
        int y;
        if (this->isAdjacentMove) {
            x = this->adjacent.move.first.first;
            y = this->adjacent.move.first.second;
        } else {
            x = this->jump.moves[0].move.first.first;
            y = this->jump.moves[0].move.first.second;
        }
        return make_pair(x, y);
    }

    pair<int, int> getEndFromMultiMove() {
        int x;
        int y;
        if (this->isAdjacentMove) {
            x = this->adjacent.move.second.first;
            y = this->adjacent.move.second.second;
        } else {
            x = this->jump.moves.back().move.second.first;
            y = this->jump.moves.back().move.second.second;
        }
        return make_pair(x, y);
    }

    string getRepr() {
        if (isJumpMove)
            return jump.getRepr();
        else
            return adjacent.getRepr();
    }
};

class Game {
public:
    /**
     * Type of game
     */
    bool type;

    /**
     * WHITE is 0
     * BLACK is 1
    */
    int color;

    /**
     * Time left for game
     */
    float timeLeft;

    /**
     * Board
     */
    vector<string> board;

    /**
     * Goal for White
     */
    set<pair<int, int>> goalW;

    /**
     * Goal for Black
     */
    set<pair<int, int>> goalB;

    Game() {
        this->board = make1D<string>(16);

        // Goal for White is topLeft
        goalW.insert(make_pair(0, 0));
        goalW.insert(make_pair(1, 0));
        goalW.insert(make_pair(2, 0));
        goalW.insert(make_pair(3, 0));
        goalW.insert(make_pair(4, 0));
        goalW.insert(make_pair(0, 1));
        goalW.insert(make_pair(1, 1));
        goalW.insert(make_pair(2, 1));
        goalW.insert(make_pair(3, 1));
        goalW.insert(make_pair(4, 1));
        goalW.insert(make_pair(0, 2));
        goalW.insert(make_pair(1, 2));
        goalW.insert(make_pair(2, 2));
        goalW.insert(make_pair(3, 2));
        goalW.insert(make_pair(0, 3));
        goalW.insert(make_pair(1, 3));
        goalW.insert(make_pair(2, 3));
        goalW.insert(make_pair(0, 4));
        goalW.insert(make_pair(1, 4));

        // Goal for White is bottomRight
        goalB.insert(make_pair(11, 15));
        goalB.insert(make_pair(12, 15));
        goalB.insert(make_pair(13, 15));
        goalB.insert(make_pair(14, 15));
        goalB.insert(make_pair(15, 15));
        goalB.insert(make_pair(11, 14));
        goalB.insert(make_pair(12, 14));
        goalB.insert(make_pair(13, 14));
        goalB.insert(make_pair(14, 14));
        goalB.insert(make_pair(15, 14));
        goalB.insert(make_pair(12, 13));
        goalB.insert(make_pair(13, 13));
        goalB.insert(make_pair(14, 13));
        goalB.insert(make_pair(15, 13));
        goalB.insert(make_pair(13, 12));
        goalB.insert(make_pair(14, 12));
        goalB.insert(make_pair(15, 12));
        goalB.insert(make_pair(14, 11));
        goalB.insert(make_pair(15, 11));
    }

    void setType(bool type) {
        Game::type = type;
    }

    void setColor(int color) {
        Game::color = color;
    }

    void setTimeLeft(float timeLeft) {
        Game::timeLeft = timeLeft;
    }
    // Define Moves String Repr
    // GetAllValidMoves()
    // Alpha Beta Pruning
    // Iterative Deepening
    // EvaluationFunction()
    // How to stop at limit?
    // Calibrate Moves/Second
};

class Reader {
public:
    string filename;

    Game game;

    Reader(string filename) {
        this->filename = filename;
    }

    void read() {
        ifstream in(this->filename);

        if (!in.is_open())
            perror("error while opening file");

        string str;
        int lineCount = 0;

        while (std::getline(in, str)) {
            // output the line
            if (lineCount == 0) {
                if (str == "SINGLE")
                    game.setType(0);
                else
                    game.setType(1);
            } else if (lineCount == 1) {
                if (str == "WHITE")
                    game.setColor(0);
                else
                    game.setColor(1);
            } else if (lineCount == 2) {
                game.setTimeLeft(stof(str));
            } else {
                game.board[lineCount - 3] = str;
            }
            lineCount++;
        }
    }

    Game getGame() {
        return this->game;
    }
};

class Triple {
public:
    int pieceID;
    int x;
    int y;

    Triple() {
        pieceID = 0;
        x = 0;
        y = 0;
    }

    Triple(int pieceID, int x, int y) {
        this->pieceID = pieceID;
        this->x = x;
        this->y = y;
    }
};

class Writer {
public:
    string filename;


    Writer(string filename) {
        this->filename = filename;
    }

    void write(string content) {
        ofstream outputFile(this->filename);
        outputFile << content;
    }
};

class State {
public:
    vector<string> board;

    // Reach Top Left
    unordered_map<int, pair<int, int>> whites;

    // Reach Bottom Right
    unordered_map<int, pair<int, int>> blacks;

    unordered_map<int, bool> whiteAtGoal;
    unordered_map<int, bool> blackAtGoal;

    State() = default;

    State(vector<string> board, unordered_map<int, pair<int, int>> whites, unordered_map<int, pair<int, int>> blacks,
          unordered_map<int, bool> whiteAtGoal,
          unordered_map<int, bool> blackAtGoal) {
        this->board = board;
        this->whites = whites;
        this->blacks = blacks;
        this->whiteAtGoal = whiteAtGoal;
        this->blackAtGoal = blackAtGoal;
    }

    /**
     * Move to a new state
     * @param type 0 if WHITE, else BLACK
     * @return Updated state with move
     */
    State movePiece(bool type, int pieceID, int xd, int yd) {

        State nextState(this->board, this->whites, this->blacks, this->whiteAtGoal, this->blackAtGoal);

        if (!type) {
            pair<int, int> piecePosition = nextState.whites[pieceID];
            nextState.whites[pieceID] = make_pair(xd, yd);
            nextState.board[piecePosition.second][piecePosition.first] = '.';
            nextState.board[yd][xd] = 'W';
        } else {
            pair<int, int> piecePosition = nextState.blacks[pieceID];
            nextState.blacks[pieceID] = make_pair(xd, yd);
            nextState.board[piecePosition.second][piecePosition.first] = '.';
            nextState.board[yd][xd] = 'B';
        }

        return nextState;
    }

};

class Node {
public:
    State state;

    pair<int, MultiMove> moveMadeFromThisNode;

    Node *parent;

    Node() {
        state = State();
    }

    Node(const Node &node) {
        this->state = node.state;
        this->moveMadeFromThisNode = node.moveMadeFromThisNode;
        this->parent = node.parent;
    }

    Node(State state) {
        this->state = state;
    }

    void setMove(pair<int, MultiMove> moveMade) {
        this->moveMadeFromThisNode = moveMade;
    }

};


int evaluation(int color, State state) {

    int v = 0;
    int h = 0;

    unordered_map<int, pair<int, int>> plus = (color == 0 ? state.whites : state.blacks);
    unordered_map<int, pair<int, int>> minus = (color == 1 ? state.whites : state.blacks);
    unordered_map<int, bool> pieceAtGoalPlus = (color == 0 ? state.whiteAtGoal : state.blackAtGoal);
    unordered_map<int, bool> pieceAtGoalMinus = (color == 1 ? state.whiteAtGoal : state.blackAtGoal);

    // Vertical Displacement + Horizontal Displacement
    if (color == 0) {
        for (auto i = plus.begin(); i != plus.end(); i++) {
            v += (15 - i->second.second);
            h += (15 - i->second.first);
        }
        for (auto i = minus.begin(); i != minus.end(); i++) {
            v -= (i->second.second);
            h -= (i->second.first);
        }
    } else {
        for (auto i = plus.begin(); i != plus.end(); i++) {
            v -= (15 - i->second.second);
            h -= (15 - i->second.first);
        }
        for (auto i = minus.begin(); i != minus.end(); i++) {
            v += (i->second.second);
            h += (i->second.first);
        }
    }

    return v + h;
}

class MoveCompare {
public:
    State state;
    int gameColor;
    int playerColor;

    MoveCompare(State state, int gameColor, int playerColor) {
        this->state = state;
        this->gameColor = gameColor;
        this->playerColor = playerColor;
    }

    bool operator()(const pair<int, MultiMove> &a, const pair<int, MultiMove> &b) {

        int ax = (a.second.isAdjacentMove ? a.second.adjacent.move.second.first
                                          : a.second.jump.moves.back().move.second.first);
        int ay = (a.second.isAdjacentMove ? a.second.adjacent.move.second.second
                                          : a.second.jump.moves.back().move.second.second);

        int bx = (b.second.isAdjacentMove ? b.second.adjacent.move.second.first
                                          : b.second.jump.moves.back().move.second.first);
        int by = (b.second.isAdjacentMove ? b.second.adjacent.move.second.second
                                          : b.second.jump.moves.back().move.second.second);

        State aNext = state.movePiece(playerColor, a.first, ax, ay);
        State bNext = state.movePiece(playerColor, b.first, bx, by);

        return evaluation(gameColor, aNext) > evaluation(gameColor, bNext);
    }
};

class Agent {
public:
    Game game;

    /**
     * Stores initial state for the problem.
     */
    State initial;

    /**
    * A compact representation of the initial state of the board
    */
    unordered_map<int, pair<int, int>> whites;
    unordered_map<int, pair<int, int>> blacks;

    /**
     * Stores a all boxes a piece has visited
     */
    map<int, set<pair<int, int>>> pieceMoves;

    Agent(Game game) {
        this->game = game;

        unordered_map<int, bool> whiteAtGoal;
        unordered_map<int, bool> blackAtGoal;

        // Add pieces to each map
        for (auto i = 0; i < 16; i++) {
            for (auto j = 0; j < 16; j++) {
                if (game.board[i][j] == 'W') {
                    whites[convert1D(j, i)] = make_pair(j, i);
                    whiteAtGoal[convert1D(i, j)] = false;
                } else if (game.board[i][j] == 'B') {
                    blacks[convert1D(j, i)] = make_pair(j, i);
                    blackAtGoal[convert1D(j, i)] = false;
                }
            }
        }

        initial = State(game.board, whites, blacks, whiteAtGoal, blackAtGoal);
        erasePieces(initial, game.color);
    }

    bool isGoal(State state) {
        bool flag = true;

        unordered_map<int, pair<int, int>> pieces = (game.color == 0 ? state.whites : state.blacks);
        set<pair<int, int>> goal = (game.color == 0 ? game.goalW : game.goalB);

        for (auto i = pieces.begin(); i != pieces.end(); i++) {
            flag &= (goal.find(i->second) != goal.end());
        }

        return flag;
    }

    bool legalMove(int x, int y) {
        return x < 16 && y < 16 && x >= 0 && y >= 0;
    }

    void addJumpMoves(vector<pair<int, MultiMove>> &jumpMoves, JumpMove state, unordered_map<int, bool> &visited,
                      vector<string> board, int pieceID, int x, int y) {

        // Check all Jump Neighbours
        for (auto i = 0; i < 8; i++) {

            int x_d = x + jumpNeighbours[i][0];
            int y_d = y + jumpNeighbours[i][1];

            int x_i = x + neighbours[i][0];
            int y_i = y + neighbours[i][1];

            bool isVisited = (visited.find(convert1D(x_d, y_d)) != visited.end());

            if (!isVisited && legalMove(x_d, y_d) && board[y_d][x_d] == '.' && board[y_i][x_i] != '.') {

                state.addMove(Move(make_pair(x, y), make_pair(x_d, y_d)));

                MultiMove m;
                m.setJumpMove(state);

                jumpMoves.emplace_back(make_pair(pieceID, m));

                visited[convert1D(x_d, y_d)] = true;

                // Update board
                board[y_d][x_d] = board[y][x];
                board[y][x] = '.';

                // Recur, don't come back to original state? Keep visited Map;
                addJumpMoves(jumpMoves, state, visited, board, pieceID, x_d, y_d);

                // Restore state
                state.moves.pop_back();

                // Reset board
                board[y][x] = board[y_d][x_d];
                board[y_d][x_d] = '.';
            }
        }
    }

    /**
     * Remove pieces with goal state achieved from whites or blacks.
     */
    void erasePieces(State &state, int type) {

        unordered_map<int, pair<int, int>> candidate = (type == 0 ? state.whites : state.blacks);
        set<pair<int, int>> goal = (type == 0 ? game.goalW : game.goalB);

        for (auto i = candidate.begin(); i != candidate.end(); i++) {
            if (goal.find(i->second) != goal.end()) {
                if (type == 0) {
                    state.whiteAtGoal[i->first] = true;
                } else {
                    state.blackAtGoal[i->first] = true;
                }
            }
        }

    }

    void
    moveGenerationSubRoutine(const Node &node, bool maxPlayer, vector<pair<int, MultiMove>> &moves, int pieceID, int x,
                             int y) {

        State state = node.state;

        unordered_map<int, pair<int, int>> candidate;
        unordered_map<int, bool> pieceAtGoal;
        set<pair<int, int>> goal;

        int playerColor = 0;

        if (maxPlayer) {
            candidate = game.color == 0 ? state.whites : state.blacks;
            pieceAtGoal = (game.color == 0 ? state.whiteAtGoal : state.blackAtGoal);
            goal = (game.color == 0 ? game.goalW : game.goalB);
            playerColor = (game.color == 0 ? 0 : 1);
        } else {
            candidate = game.color == 0 ? state.blacks : state.whites;
            pieceAtGoal = (game.color == 0 ? state.blackAtGoal : state.whiteAtGoal);
            goal = (game.color == 0 ? game.goalB : game.goalW);
            playerColor = (game.color == 0 ? 1 : 0);
        }

        // whites / blacks stores original positions of W/B pieces
        vector<pair<int, MultiMove>> jumpMoves;
        JumpMove jumpState;
        unordered_map<int, bool> visited;
        visited[convert1D(x, y)] = true;
        addJumpMoves(jumpMoves, jumpState, visited, state.board, pieceID, x, y);

        if (!jumpMoves.empty()) {
            if (!pieceAtGoal[pieceID]) {
                for (auto i = 0; i < jumpMoves.size(); i++) {
                    int x_final = jumpMoves[i].second.jump.moves.back().move.second.first;
                    int y_final = jumpMoves[i].second.jump.moves.back().move.second.second;
                    if (pieceMoves[pieceID].find(make_pair(x_final, y_final)) == pieceMoves[pieceID].end())
                        moves.emplace_back(jumpMoves[i]);
                }
            } else {
                for (auto i = 0; i < jumpMoves.size(); i++) {
                    int x_final = jumpMoves[i].second.jump.moves.back().move.second.first;
                    int y_final = jumpMoves[i].second.jump.moves.back().move.second.second;
                    if (goal.find(make_pair(x_final, y_final)) != goal.end()) {
                        if (playerColor == 0 && x_final <= x && y_final <= y) {
                            if (pieceMoves[pieceID].find(make_pair(x_final, y_final)) == pieceMoves[pieceID].end()) {
                                moves.emplace_back(jumpMoves[i]);
                            }
                        } else if (playerColor == 1 && x_final >= x && y_final >= y) {
                            if (pieceMoves[pieceID].find(make_pair(x_final, y_final)) == pieceMoves[pieceID].end()) {
                                moves.emplace_back(jumpMoves[i]);
                            }
                        }
                    }
                }
            }
        }
        // Adding Non Jump Moves
        // Add moves within goal if piece at goal
        // Else add all moves
        for (auto idx = 0; idx < 8; idx++) {

            int x_d = x + neighbours[idx][0];
            int y_d = y + neighbours[idx][1];

            if (pieceMoves[pieceID].find(make_pair(x_d, y_d)) != pieceMoves[pieceID].end())
                continue;

            if (legalMove(x_d, y_d) && state.board[y_d][x_d] == '.') {

                if (!pieceAtGoal[pieceID]) {
                    MultiMove temp;
                    temp.setAdjacentMove(Move(x, y, x_d, y_d));
                    moves.emplace_back(make_pair(pieceID, temp));
                } else if (goal.find(make_pair(x_d, y_d)) != goal.end()) {
                    MultiMove temp;
                    temp.setAdjacentMove(Move(x, y, x_d, y_d));

                    if (playerColor == 0 && x_d <= x && y_d <= y)
                        moves.emplace_back(make_pair(pieceID, temp));
                    else if (playerColor == 1 && x_d >= x && y_d >= y)
                        moves.emplace_back(make_pair(pieceID, temp));
                }
            }
        }

    }

    vector<pair<int, MultiMove>> generateMoves(Node node, bool maxPlayer) {

        // We can at minimum have 19*8 moves + 19 * (0 or more Jump Moves)
        // We should be adding Killer Moves first. Can maybe just return top k moves. Sort by eval score.
        // If a piece is at goal, then don't generate moves for the piece.
        State state = node.state;
        set<pair<int, int>> camp;
        unordered_map<int, pair<int, int>> candidate;
        vector<pair<int, MultiMove>> moves;

        if (maxPlayer) {
            candidate = game.color == 0 ? state.whites : state.blacks;
            camp = (game.color == 1 ? game.goalW : game.goalB);
        } else {
            candidate = game.color == 0 ? state.blacks : state.whites;
            camp = (game.color == 1 ? game.goalB : game.goalW);
        }

        // Check if pieces still in base camp
        bool arePiecesinBaseCamp = false;
        for (auto i = candidate.begin(); i != candidate.end(); i++) {
            if (camp.find(i->second) != camp.end()) {
                moveGenerationSubRoutine(node, maxPlayer, moves, i->first, i->second.first, i->second.second);
            }
        }

        // Ensures pieces inside camp are moved first.
        if (moves.size() != 0) {
            return moves;
        } else {
            for (auto i = candidate.begin(); i != candidate.end(); i++) {
                moveGenerationSubRoutine(node, maxPlayer, moves, i->first, i->second.first, i->second.second);
            }
        }

        return moves;
    }

    Node alphaBetaMove(Node &node, int depth, bool maxPlayer, int alpha, int beta, int startTime) {

        if (depth == 0 || isGoal(node.state))
            return node;

        if (maxPlayer) {
            int v = INT_MIN;
            Node candidateNode = Node();
            vector<pair<int, MultiMove>> moves = generateMoves(node, maxPlayer);

            // Sort Moves by eval metric
            sort(moves.begin(), moves.end(), MoveCompare(node.state, game.color, game.color));

            // A Heuristic
            moves.resize(min(int(moves.size()), 10));

            for (auto i = 0; i < moves.size(); i++) {
                // Cutoff at maxTime - 10 seconds
                if (game.timeLeft - (clock() / CLOCKS_PER_SEC - startTime) <= 1) {
                    break;
                } else {
                    Node nextNode;

                    if (moves[i].second.isAdjacentMove) {
                        int x_d = moves[i].second.adjacent.move.second.first;
                        int y_d = moves[i].second.adjacent.move.second.second;
                        nextNode = Node(node.state.movePiece(game.color, moves[i].first, x_d, y_d));
                        nextNode.setMove(moves[i]);
                    } else {
                        int x_d = moves[i].second.jump.moves.back().move.second.first;
                        int y_d = moves[i].second.jump.moves.back().move.second.second;
                        nextNode = Node(node.state.movePiece(game.color, moves[i].first, x_d, y_d));
                        nextNode.setMove(moves[i]);
                    }

                    int val = evaluation(game.color,
                                         alphaBetaMove(nextNode, depth - 1, false, alpha, beta,
                                                       startTime).state);
                    if (val > v) {
                        candidateNode = nextNode;
                        v = val;
                    }

                    alpha = max(alpha, v);
                    if (beta <= alpha)
                        break;
                }

            }
            return candidateNode;
        } else {
            int v = INT_MAX;
            Node candidateNode = Node();
            vector<pair<int, MultiMove>> moves = generateMoves(node, maxPlayer);

            int other_color = (game.color == 1 ? 0 : 1);

            // Sort Moves by eval metric
            sort(moves.begin(), moves.end(), MoveCompare(node.state, game.color, other_color));

            // A Heuristic
            moves.resize(min(int(moves.size()), 10));

            for (auto i = 0; i < moves.size(); i++) {
                // Cutoff at maxTime - 10 seconds
                if (game.timeLeft - (clock() / CLOCKS_PER_SEC - startTime) <= 10) {
                    break;
                } else {
                    Node nextNode;

                    if (moves[i].second.isAdjacentMove) {
                        int x_d = moves[i].second.adjacent.move.second.first;
                        int y_d = moves[i].second.adjacent.move.second.second;
                        nextNode = Node(node.state.movePiece(other_color, moves[i].first, x_d, y_d));
                        nextNode.setMove(moves[i]);
                    } else {
                        int x_d = moves[i].second.jump.moves.back().move.second.first;
                        int y_d = moves[i].second.jump.moves.back().move.second.second;
                        nextNode = Node(node.state.movePiece(other_color, moves[i].first, x_d, y_d));
                        nextNode.setMove(moves[i]);
                    }

                    int val = evaluation(game.color,
                                         alphaBetaMove(nextNode, depth - 1, true, alpha, beta,
                                                       startTime).state);

                    if (val < v) {
                        candidateNode = nextNode;
                        v = val;
                    }

                    beta = min(beta, v);
                    if (beta <= alpha)
                        break;
                }
            }
            return candidateNode;
        }
    }

    Node alphaBetaGame(Node node, int depth, bool maxPlayer, int alpha, int beta, int startTime) {

        if (depth == 0 || isGoal(node.state))
            return node;

        if (maxPlayer) {
            int v = INT_MIN;
            Node candidateNode = Node();

            vector<Triple> moves;// = generateMoves(node.state, maxPlayer);

            // Sort Moves by eval metric
            //sort(moves.begin(),moves.end());

            // A Heuristic
            moves.resize(min(int(moves.size()), 10));

            for (auto i = 0; i < moves.size(); i++) {
                // Cutoff at maxTime - 10 seconds
                if (clock() / CLOCKS_PER_SEC - startTime >= 5) {
                    break;
                } else {
                    Node nextNode = Node(
                            node.state.movePiece(game.color, moves[i].pieceID, moves[i].x, moves[i].y));
                    int val = evaluation(game.color,
                                         alphaBetaGame(nextNode, depth - 1, !maxPlayer, alpha, beta,
                                                       startTime).state);

                    if (val > v) {
                        candidateNode = nextNode;
                        v = val;
                    }

                    alpha = max(alpha, v);
                    if (beta <= alpha)
                        break;
                }

            }
            return candidateNode;
        } else {
            int v = INT_MAX;
            Node candidateNode = Node();
            vector<Triple> moves; //= generateMoves(node.state, maxPlayer);
            int other_color = (game.color == 1 ? 0 : 1);

            // Sort Moves by eval metric
            MoveCompare c = MoveCompare(node.state, game.color, other_color);
            //sort(moves.begin(),moves.end());

            // A Heuristic
            moves.resize(min(int(moves.size()), 10));

            for (auto i = 0; i < moves.size(); i++) {
                // Cutoff at maxTime - 10 seconds
                if (clock() / CLOCKS_PER_SEC - startTime >= 5) {
                    break;
                } else {
                    Node nextNode = Node(
                            node.state.movePiece(other_color, moves[i].pieceID, moves[i].x, moves[i].y));

                    int val = evaluation(game.color,
                                         alphaBetaGame(nextNode, depth - 1, !maxPlayer, alpha, beta,
                                                       startTime).state);

                    if (val < v) {
                        candidateNode = nextNode;
                        v = val;
                    }

                    beta = min(beta, v);
                    if (beta <= alpha)
                        break;
                }
            }
            return candidateNode;
        }
    }

    void singleMovePlayer() {
        Node initial = Node(this->initial);
        int start = clock() / CLOCKS_PER_SEC;
        Node result = alphaBetaMove(initial, 2, true, INT_MIN, INT_MAX, start);
        printBoard(result.state.board);

        // Write Output
        Writer writer("output.txt");
        writer.write(result.moveMadeFromThisNode.second.getRepr());
        cout << result.moveMadeFromThisNode.second.getRepr() << endl;
    }

    void simulateGame() {
        Node init = Node(this->initial);
        Node *p1 = NULL;
        Node *p2 = NULL;

        for (auto i = 0; i < 500; i++) {
            cout << i << endl;
            if (i % 2 == 0) {
                // Play
                init.parent = p1;
                game.color = 0;

                int start = clock() / CLOCKS_PER_SEC;
                cout << "Start Time: " << start << endl;
                Node result = alphaBetaMove(init, 1, true, INT_MIN, INT_MAX, start);

                p1 = &init;

                // Add piece origin to history
                pair<int, int> startCoord = result.moveMadeFromThisNode.second.getStartFromMultiMove();
                pieceMoves[result.moveMadeFromThisNode.first].insert(startCoord);

                erasePieces(result.state, game.color);

                int end = clock() / CLOCKS_PER_SEC;
                init = result;

                printBoard(result.state.board);

                if (isGoal(result.state))
                    break;

            } else {
                // Play opponent
                init.parent = p2;

                int start = clock() / CLOCKS_PER_SEC;
                cout << "Start Time: " << start << endl;
                game.color = 1;
                Node result = alphaBetaMove(init, 1, true, INT_MIN, INT_MAX, start);

                p2 = &init;

                // Add piece origin to history
                pair<int, int> startCoord = result.moveMadeFromThisNode.second.getStartFromMultiMove();
                pieceMoves[result.moveMadeFromThisNode.first].insert(startCoord);

                erasePieces(result.state, game.color);

                int end = clock() / CLOCKS_PER_SEC;
                init = result;

                printBoard(result.state.board);

                if (isGoal(result.state))
                    break;

            }
        }
    }

};

int main() {
    Reader r("input3.txt");
    r.read();

    Agent a(r.game);

    if (r.game.type == 0) {

        vector<pair<int, MultiMove>> m = a.generateMoves(a.initial, true);

        for (auto i = 0; i < m.size(); i++) {
            cout << m[i].second.getRepr() << endl;
        }
        //a.singleMovePlayer();
    } else {
        a.simulateGame();
    }

    return 0;
}