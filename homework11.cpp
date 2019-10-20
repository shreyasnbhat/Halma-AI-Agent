#include <bits/stdc++.h>
#include <time.h>

using namespace std;

// x is col#, y is row#
#define convert1D(x, y) y * 16 + x
#define ROW(val) val / 16
#define COL(val) val % 16

int lineSide(int xm,int ym) {
    return (xm - ym + 4 >= 0) && (xm - ym - 4 <= 0);
}

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
        int x = -1;
        int y = -1;
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
        int x = -1;
        int y = -1;
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

            str.erase( std::remove(str.begin(), str.end(), '\r'), str.end() );

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

    int excess = 0;

    unordered_map<int, pair<int, int>> plus = (color == 0 ? state.whites : state.blacks);
    unordered_map<int, pair<int, int>> minus = (color == 1 ? state.whites : state.blacks);
    unordered_map<int, bool> pieceAtGoalPlus = (color == 0 ? state.whiteAtGoal : state.blackAtGoal);
    unordered_map<int, bool> pieceAtGoalMinus = (color == 1 ? state.whiteAtGoal : state.blackAtGoal);

    // Vertical Displacement + Horizontal Displacement
    if (color == 0) {
        for (auto i = plus.begin(); i != plus.end(); i++) {
            bool withinLine = lineSide(i->second.first,i->second.second);
            v += (withinLine ? 2 * (15 - i->second.second): 15 - i->second.second);
            h += (withinLine ? 2 * (15 - i->second.first): 15 - i->second.first);
            excess += sqrt(i->second.second * i->second.second + i->second.first * i->second.first);
        }
        for (auto i = minus.begin(); i != minus.end(); i++) {
            bool withinLine = lineSide(i->second.first,i->second.second);
            v -= (withinLine ? i->second.second: 2 * i->second.second);
            h -= (withinLine ? i->second.first:  2 * i->second.first);
            excess -= sqrt((15 - i->second.second) * (15 - i->second.second) + (15 - i->second.first) * (15 - i->second.first));
        }
    } else {
        for (auto i = plus.begin(); i != plus.end(); i++) {
            bool withinLine = lineSide(i->second.first,i->second.second);
            v -= (withinLine ? 15 - i->second.second: 2 * (15 - i->second.second));
            h -= (withinLine ? 15 - i->second.first:  2 * (15 - i->second.first));
            excess -= sqrt(i->second.second * i->second.second + i->second.first * i->second.first);
        }
        for (auto i = minus.begin(); i != minus.end(); i++) {
            bool withinLine = lineSide(i->second.first,i->second.second);
            v += (withinLine ? 2 * i->second.second: i->second.second);
            h += (withinLine ? 2 * i->second.first: i->second.first);
            excess += sqrt((15 - i->second.second) * (15 - i->second.second) + (15 - i->second.first) * (15 - i->second.first));
        }
    }

    return v + h + excess;
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

    bool operator()(pair<int, MultiMove> &a, pair<int, MultiMove> &b) {

        pair<int, int> aCoord = a.second.getEndFromMultiMove();
        pair<int, int> bCoord = b.second.getEndFromMultiMove();

        State aNext = state.movePiece(playerColor, a.first, aCoord.first, aCoord.second);
        State bNext = state.movePiece(playerColor, b.first, bCoord.first, bCoord.second);

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
     * Potentially in playData.txt
     */
    map<int, set<pair<int, int>>> pieceMoves;

    Agent(Game game) {
        this->game = game;

        unordered_map<int, bool> whiteAtGoal;
        unordered_map<int, bool> blackAtGoal;

        vector<int> whitePieceIDs = {};
        vector<int> blackPieceIDs = {};

        ifstream in("playdata.txt");

        // Single games don't read playdata.txt
        if (!in.is_open() || game.type == 0) {
            perror("error while opening file");
            // Add pieces to each map
            for (auto i = 0; i < 16; i++) {
                for (auto j = 0; j < 16; j++) {
                    if (game.board[i][j] == 'W') {
                        whites[convert1D(j, i)] = make_pair(j, i);
                        //whiteAtGoal[convert1D(i, j)] = false;
                    } else if (game.board[i][j] == 'B') {
                        blacks[convert1D(j, i)] = make_pair(j, i);
                        //blackAtGoal[convert1D(j, i)] = false;
                    }
                }
            }
        } else {
            readPlayData();
        }

        erasePieces(initial, 0);
        erasePieces(initial, 1);
        initial = State(game.board, whites, blacks, whiteAtGoal, blackAtGoal);
    }

    void writePlayData() {

        string content;

        for(auto i = whites.begin(); i!= whites.end(); i++) {
            int pieceID = i->first;
            pair<int,int> s = i->second;
            content.append(to_string(pieceID) + " ");
            content.append(to_string(convert1D(s.first, s.second)) + "\n");
        }

        content.append("BREAK\n");

        for(auto k = blacks.begin(); k!= blacks.end(); k++) {
            int pieceID = k->first;
            pair<int,int> s = k->second;
            content.append(to_string(pieceID) + " ");
            content.append(to_string(convert1D(s.first, s.second)) + "\n");
        }

        content.append("BREAK\n");

        for(auto i = pieceMoves.begin(); i!= pieceMoves.end(); i++) {
            int key = i->first;
            set<pair<int,int>> s = i->second;

            if(s.size() != 0) {
                content.append(to_string(key) + " ");
                int size = s.size();
                int ctr = 0;

                for (auto k = s.begin(); k != s.end(); k++) {
                    if (ctr < size - 1)
                        content.append(to_string(convert1D(k->first, k->second)) + " ");
                    else
                        content.append(to_string(convert1D(k->first, k->second)) + "\n");
                    ctr++;
                }
            }
        }

        Writer w("playdata.txt");
        w.write(content);
    }

    void readPlayData() {
        ifstream in("playdata.txt");

        if (!in.is_open()) {
            perror("error while opening file");
            return;
        }
        int ctr = 1;
        string str;
        while (std::getline(in, str)) {
            istringstream ss(str);
            string word;
            ss >> word;
            int key = atoi(word.c_str());

            do {
                string word;
                ss >> word;
                // Print the read word
                int val = atoi(word.c_str());
                int x = COL(val);
                int y = ROW(val);

                if(word.size() != 0) {
                    if (ctr <= 19) {
                        //cout << "White: " << key << " " << x << " " << y << endl;
                        whites[key] = make_pair(x, y);
                        break;
                    } else if (ctr > 20 && ctr <= 39) {
                        //cout << "Black: " << key << " " << x << " " << y << endl;
                        blacks[key] = make_pair(x, y);
                        break;
                    } else if(ctr > 40){
                        pieceMoves[key].insert(make_pair(x, y));
                        //cout << "Key: " << key << " " << x << " " << y << endl;
                    }
                }
            } while (ss);
            ctr++;
        }
    }

    // Need to handle case where blacks don't move.
    bool isGoal(State state) {

        unordered_map<int, pair<int, int>> playerPieces = (game.color == 0 ? state.whites : state.blacks);
        unordered_map<int, pair<int, int>> opponentPieces = (game.color == 0 ? state.blacks : state.whites);
        set<pair<int,int>> opponentsInBaseCamp;
        set<pair<int, int>> goal = (game.color == 0 ? game.goalW : game.goalB);

        for (auto i = opponentPieces.begin(); i != opponentPieces.end(); i++) {
            if(goal.find(i->second) != goal.end()) {
                opponentsInBaseCamp.insert(i->second);
            }
        }

        if(opponentsInBaseCamp.size() == 19)
            return false;

        set<pair<int, int>> emptySlotsAtGoal;
        set_difference(goal.begin(), goal.end(), opponentsInBaseCamp.begin(), opponentsInBaseCamp.end(),
                       std::inserter(emptySlotsAtGoal, emptySlotsAtGoal.end()));

        for (auto i = playerPieces.begin(); i != playerPieces.end(); i++) {
            if(emptySlotsAtGoal.find(i->second) != emptySlotsAtGoal.end()) {
                emptySlotsAtGoal.erase(i->second);
            }
        }

        return (emptySlotsAtGoal.empty());
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

                // Recur
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
        set<pair<int, int>> baseCamp;

        int playerColor = 0;

        if (maxPlayer) {
            candidate = game.color == 0 ? state.whites : state.blacks;
            pieceAtGoal = (game.color == 0 ? state.whiteAtGoal : state.blackAtGoal);
            goal = (game.color == 0 ? game.goalW : game.goalB);
            baseCamp = (game.color == 0 ? game.goalB : game.goalW);
            playerColor = (game.color == 0 ? 0 : 1);
        } else {
            candidate = game.color == 0 ? state.blacks : state.whites;
            pieceAtGoal = (game.color == 0 ? state.blackAtGoal : state.whiteAtGoal);
            goal = (game.color == 0 ? game.goalB : game.goalW);
            baseCamp = (game.color == 0 ? game.goalW : game.goalB);
            playerColor = (game.color == 0 ? 1 : 0);
        }

        // whites / blacks stores original positions of W/B pieces
        vector<pair<int, MultiMove>> jumpMoves;
        JumpMove jumpState;
        unordered_map<int, bool> visited;
        visited[convert1D(x, y)] = true;
        addJumpMoves(jumpMoves, jumpState, visited, state.board, pieceID, x, y);

        pair<int,int> originalCoord = make_pair(x,y);

        if (!jumpMoves.empty()) {
            if (!pieceAtGoal[pieceID]) {
                for (auto i = 0; i < jumpMoves.size(); i++) {
                    pair<int,int> coord = jumpMoves[i].second.getEndFromMultiMove();

                    // Outside to basecamp is restricted
                    if(baseCamp.find(originalCoord) == baseCamp.end() && baseCamp.find(coord) != baseCamp.end())
                        continue;

                    // If coordinate starts and ends in Base Camp restrict movement options.
                    if(baseCamp.find(originalCoord) != baseCamp.end() && baseCamp.find(coord) != baseCamp.end()) {
                        if(playerColor == 0 && coord.first <= x && coord.second <= y) {
                            moves.emplace_back(jumpMoves[i]);
                        } else if(playerColor == 1 && coord.first >= x && coord.second >= y) {
                            moves.emplace_back(jumpMoves[i]);
                        }
                    } else if(baseCamp.find(originalCoord) != baseCamp.end() && baseCamp.find(coord) == baseCamp.end()) {
                        // Base Camp to outside move
                        moves.emplace_back(jumpMoves[i]);
                    } else if(pieceMoves[pieceID].find(coord) == pieceMoves[pieceID].end()) {
                        moves.emplace_back(jumpMoves[i]);
                    }
                }
            } else {
                for (auto i = 0; i < jumpMoves.size(); i++) {
                    pair<int,int> coord = jumpMoves[i].second.getEndFromMultiMove();

                    // Outside to basecamp is restricted , this assumes a big jump from opposing camp to base camp
                    if(baseCamp.find(originalCoord) == baseCamp.end() && baseCamp.find(coord) != baseCamp.end())
                        continue;

                    if (goal.find(coord) != goal.end()) {
                        if (playerColor == 0 && coord.first <= x && coord.second <= y) {
                            if (pieceMoves[pieceID].find(coord) == pieceMoves[pieceID].end()) {
                                moves.emplace_back(jumpMoves[i]);
                            }
                        } else if (playerColor == 1 && coord.first >= x && coord.second >= y) {
                            if (pieceMoves[pieceID].find(coord) == pieceMoves[pieceID].end()) {
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

            pair<int,int> coord = make_pair(x_d,y_d);

            MultiMove temp;
            temp.setAdjacentMove(Move(x, y, x_d, y_d));

            if (pieceMoves[pieceID].find(coord) != pieceMoves[pieceID].end())
                continue;

            // Outside to basecamp is restricted
            if(baseCamp.find(originalCoord) == baseCamp.end() && baseCamp.find(coord) != baseCamp.end())
                continue;

            if (legalMove(x_d, y_d) && state.board[y_d][x_d] == '.') {

                if (!pieceAtGoal[pieceID]) {
                    if(baseCamp.find(originalCoord) != baseCamp.end() && baseCamp.find(coord) != baseCamp.end()) {
                        if (playerColor == 0 && coord.first <= x && coord.second <= y) {
                            moves.emplace_back(make_pair(pieceID, temp));
                        } else if (playerColor == 1 && coord.first >= x && coord.second >= y) {
                            moves.emplace_back(make_pair(pieceID, temp));
                        }
                    } else {
                        moves.emplace_back(make_pair(pieceID, temp));
                    }
                } else if (goal.find(coord) != goal.end()) {
                    if (playerColor == 0 && x_d <= x && y_d <= y)
                        moves.emplace_back(make_pair(pieceID, temp));
                    else if (playerColor == 1 && x_d >= x && y_d >= y)
                        moves.emplace_back(make_pair(pieceID, temp));
                }
            }
        }

        //cout << "Got moves for PieceID: " << pieceID << endl;
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

                    pair<int, int> coord = moves[i].second.getEndFromMultiMove();
                    nextNode = Node(node.state.movePiece(game.color, moves[i].first, coord.first, coord.second));
                    nextNode.setMove(moves[i]);

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
                if (game.timeLeft - (clock() / CLOCKS_PER_SEC - startTime) <= 1) {
                    break;
                } else {
                    Node nextNode;

                    pair<int, int> coord = moves[i].second.getEndFromMultiMove();
                    nextNode = Node(node.state.movePiece(other_color, moves[i].first, coord.first, coord.second));
                    nextNode.setMove(moves[i]);

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

    void singleMovePlayer(int depth) {
        Node initial = Node(this->initial);

        if (isGoal(initial.state))
            return;

        int start = clock() / CLOCKS_PER_SEC;
        Node result = alphaBetaMove(initial, depth, true, INT_MIN, INT_MAX, start);

        // Adding pieceHistory
        int pieceID = result.moveMadeFromThisNode.first;
        pair<int, int> startCoord = result.moveMadeFromThisNode.second.getStartFromMultiMove();
        pieceMoves[pieceID].insert(startCoord);

        printBoard(result.state.board);
        cout << endl;

        whites = result.state.whites;
        blacks = result.state.blacks;

        // Write Output
        Writer writer("output.txt");
        writer.write(result.moveMadeFromThisNode.second.getRepr());
    }

};

int main() {
    Reader r("tests/input8.txt");
    r.read();

    Agent a(r.game);

    if (r.game.type == 0) {
        a.singleMovePlayer(1);
    } else {
        a.singleMovePlayer(2);
        a.writePlayData();
    }


    return 0;
}