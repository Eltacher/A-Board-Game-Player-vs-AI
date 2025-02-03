package BoardGameG2DPlayervsAI;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents the type of piece on the board.
*/
enum Piece 
{
    NONE, PLAYER, AI
}

/**
 * Represents the phases of the game.
*/
enum GameState 
{
    AI_FIRST_TURN, AI_SECOND_TURN,
    PLAYER_FIRST_TURN, PLAYER_SECOND_TURN
}

/**
 * Helper class to store move information.
*/
class Move 
{
    int fromRow, fromCol;
    int toRow, toCol;

    public Move(int fromRow, int fromCol, int toRow, int toCol) 
    {
        this.fromRow = fromRow;
        this.fromCol = fromCol;
        this.toRow = toRow;
        this.toCol = toCol;
    }

    @Override
    public String toString() 
    {
        return String.format("Move[from=(%d,%d) to=(%d,%d)]", fromRow, fromCol, toRow, toCol);
    }
}

public class BoardGameG2DwithAI extends JPanel implements MouseListener 
{

    private final Piece[][] board;    // 7x7 board
    private final int cellSize;       // Size of each cell for rendering

    private int moveCount = 0;        // Total moves made
    private GameState currentState;   // Current phase of the game
    private String gameResult;        // Final game result message
    private boolean isGameOver = false;

    // For tracking piece selection and last move
    private Point selectedPiecePosition; 
    private Point lastMovedPiecePosition;

    private boolean captureOccurred = false; 
    private int playerMovesMade = 0;  

    // Colors for highlighting
    private static final Color SELECTION_COLOR  = new Color(255, 255, 0, 100);
    private static final Color LAST_MOVE_COLOR  = new Color(0, 255, 0, 100);
    private static final Color VALID_MOVE_COLOR = new Color(0, 255, 0, 50);

    public BoardGameG2DwithAI(int cellSize) {
        this.cellSize = cellSize;
        this.board = new Piece[7][7];
        this.currentState = GameState.AI_FIRST_TURN;
        this.selectedPiecePosition = null;
        this.lastMovedPiecePosition = null;

        initializeBoard();
        addMouseListener(this);

        // Create a one-shot timer (1000ms) so the AI makes its first move once the UI is visible
        Timer timer = new Timer(1000, e -> runAIInBackground());
        timer.setRepeats(false);
        timer.start();

        System.out.println("BoardGame initialized. AI's first move will run shortly...");
    }

    /**
     * Initializes the 7x7 board with starting positions for both PLAYER and AI.
    */
    private void initializeBoard() 
    {
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                board[row][col] = Piece.NONE;
            }
        }

        // AI pieces
        board[0][0] = Piece.AI;
        board[2][0] = Piece.AI;
        board[4][6] = Piece.AI;
        board[6][6] = Piece.AI;

        // Player pieces
        board[0][6] = Piece.PLAYER;
        board[2][6] = Piece.PLAYER;
        board[4][0] = Piece.PLAYER;
        board[6][0] = Piece.PLAYER;
    }

    // ========================== VALIDATION & MOVE HELPERS =========================

    /**
     * Checks if the move from (fromRow, fromCol) to (toRow, toCol) is valid:
        - The destination must be on the board and empty.
        - The move must be to an adjacent cell (Manhattan distance == 1).
    */
    private boolean isValidMove(int fromRow, int fromCol, int toRow, int toCol, Piece[][] tempBoard) 
    {
        if (toRow < 0 || toRow >= 7 || toCol < 0 || toCol >= 7) 
        {
            return false;
        }
        int rowDiff = Math.abs(fromRow - toRow);
        int colDiff = Math.abs(fromCol - toCol);
        return (rowDiff + colDiff) == 1 && (tempBoard[toRow][toCol] == Piece.NONE);
    }

    /**
     * Retrieves valid moves for highlighting (for the currently selected piece).
    */
    private List<Point> getValidMoves(int row, int col) 
    {
        List<Point> validMoves = new ArrayList<>();
        int[][] directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };
        for (int[] dir : directions)
         {
            int newRow = row + dir[0];
            int newCol = col + dir[1];
            if (isValidMove(row, col, newRow, newCol, board)) 
            {
                validMoves.add(new Point(newRow, newCol));
            }
        }
        return validMoves;
    }

    /**
     * Counts how many pieces of the given type are on the board.
    */
    private int countPieces(Piece player, Piece[][] tempBoard) 
    {
        int count = 0;
        for (int row = 0; row < 7; row++)
        {
            for (int col = 0; col < 7; col++) 
            {
                if (tempBoard[row][col] == player) 
                {
                    count++;
                }
            }
        }
        return count;
    }

    /**
     * Returns all possible moves for the given player on the provided board state.
    */
    private List<Move> getAllPossibleMoves(Piece[][] tempBoard, Piece currentPlayer) 
    {
        List<Move> moves = new ArrayList<>();
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                if (tempBoard[row][col] == currentPlayer) 
                {
                    int[][] directions = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } };
                    for (int[] dir : directions) 
                    {
                        int newRow = row + dir[0];
                        int newCol = col + dir[1];
                        if (isValidMove(row, col, newRow, newCol, tempBoard)) 
                        {
                            moves.add(new Move(row, col, newRow, newCol));
                        }
                    }
                }
            }
        }
        return moves;
    }

    // ========================== SIMULATION & CAPTURES =========================

    /**
     * Simulates a given move on a temporary board, then performs capture logic.
    */
    private void simulateMove(Piece[][] tempBoard, Move move) 
    {
        // Make the move
        tempBoard[move.toRow][move.toCol] = tempBoard[move.fromRow][move.fromCol];
        tempBoard[move.fromRow][move.fromCol] = Piece.NONE;

        // After the move, check captures
        List<Point> piecesToRemove = new ArrayList<>();
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                if (tempBoard[row][col] != Piece.NONE) 
                {
                    Piece currentPiece = tempBoard[row][col];
                    Piece opponentPiece = (currentPiece == Piece.PLAYER) ? Piece.AI : Piece.PLAYER;

                    // "Wall" captures
                    checkCaptureWithWall(tempBoard, row, col, -1, 0, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(tempBoard, row, col, 1, 0, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(tempBoard, row, col, 0, -1, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(tempBoard, row, col, 0, 1, opponentPiece, piecesToRemove);

                    // "Sandwich" captures
                    checkSandwichCapture(tempBoard, row, col, 1, 0, piecesToRemove);
                    checkSandwichCapture(tempBoard, row, col, -1, 0, piecesToRemove);
                    checkSandwichCapture(tempBoard, row, col, 0, 1, piecesToRemove);
                    checkSandwichCapture(tempBoard, row, col, 0, -1, piecesToRemove);
                }
            }
        }

        // Remove captured pieces
        for (Point p : piecesToRemove) 
        {
            tempBoard[p.x][p.y] = Piece.NONE;
        }
    }

    /**
     * Checks "wall" capture in a specific direction (dRow, dCol).
    */
    private void checkCaptureWithWall(Piece[][] tempBoard, int row, int col, int dRow, int dCol,  Piece opponentPiece, List<Point> toRemove) 
    {
        int r = row + dRow;
        int c = col + dCol;
        List<Point> potentialCaptures = new ArrayList<>();

        while (r >= 0 && r < 7 && c >= 0 && c < 7 && tempBoard[r][c] == opponentPiece) 
        {
            potentialCaptures.add(new Point(r, c));
            r += dRow;
            c += dCol;
        }

        // If we reach outside the board or back to the same player's piece, remove the opponent's pieces
        if ((r < 0 || r >= 7 || c < 0 || c >= 7) || (r >= 0 && r < 7 && c >= 0 && c < 7 && tempBoard[r][c] == tempBoard[row][col])) 
        {
            toRemove.addAll(potentialCaptures);
        }
    }

    /**
     * Checks "sandwich" capture in a specific direction (dRow, dCol).
    */
    private void checkSandwichCapture(Piece[][] tempBoard, int row, int col, int dRow, int dCol, List<Point> toRemove) 
    {
        Piece currentPiece = tempBoard[row][col];
        Piece opponentPiece = (currentPiece == Piece.PLAYER) ? Piece.AI : Piece.PLAYER;

        int r = row + dRow;
        int c = col + dCol;
        List<Point> potentialCaptures = new ArrayList<>();

        while (r >= 0 && r < 7 && c >= 0 && c < 7) 
        {
            if (tempBoard[r][c] == opponentPiece) 
            {
                potentialCaptures.add(new Point(r, c));
            }
            else if (tempBoard[r][c] == currentPiece) 
            {
                // Everything between the same player's pieces is captured
                toRemove.addAll(potentialCaptures);
                return;
            }
            else 
            {
                break;
            }
            r += dRow;
            c += dCol;
        }
    }

    // ========================== EVALUATION & MINIMAX =========================

    /**
     * Checks if the given board state is "terminal":
        - If AI or Player has 0 pieces
        - Or if the search depth has reached 0.
    */
    private boolean isTerminalState(Piece[][] tempBoard, int depth) 
    {
        int aiCount     = countPieces(Piece.AI, tempBoard);
        int playerCount = countPieces(Piece.PLAYER, tempBoard);
        return (aiCount == 0 || playerCount == 0 || depth <= 0);
    }

    /**
     * Creates a deep copy of the 7x7 board array.
    */
    private Piece[][] copyBoard(Piece[][] original) 
    {
        Piece[][] copy = new Piece[7][7];
        for (int i = 0; i < 7; i++) {
            System.arraycopy(original[i], 0, copy[i], 0, 7);
        }
        return copy;
    }

    /**
     * Minimax with Alpha-Beta pruning, taking a precomputed moves list for the current player (subset if needed).
     * Returns [bestScore, bestMoveIndex].
    */
    private int[] minimaxAlphaBeta(Piece[][] tempBoard, List<Move> precomputedMoves, int depth, int alpha, int beta, boolean maximizingPlayer) 
    {

        // Terminal or depth exhausted
        if (isTerminalState(tempBoard, depth)) 
        {
            int eval = evaluateBoard(tempBoard);
            return new int[] { eval, -1 };
        }

        // If no moves, treat as terminal
        if (precomputedMoves.isEmpty()) 
        {
            int eval = evaluateBoard(tempBoard);
            return new int[] { eval, -1 };
        }

        int bestMoveIndex = -1;

        if (maximizingPlayer) 
        {
            int maxEval = Integer.MIN_VALUE;
            for (int i = 0; i < precomputedMoves.size(); i++) 
            {
                Move move = precomputedMoves.get(i);
                Piece[][] copy = copyBoard(tempBoard);
                simulateMove(copy, move);

                // Next moves for the opponent
                List<Move> nextMoves = getAllPossibleMoves(copy, Piece.PLAYER);
                int eval = minimaxAlphaBeta(copy, nextMoves, depth - 1, alpha, beta, false)[0];

                if (eval > maxEval) 
                {
                    maxEval = eval;
                    bestMoveIndex = i;
                }
                alpha = Math.max(alpha, maxEval);
                if (beta <= alpha) 
                {
                    break; 
                }
            }
            return new int[] { maxEval, bestMoveIndex };
        }
        else 
        {
            int minEval = Integer.MAX_VALUE;
            for (int i = 0; i < precomputedMoves.size(); i++) 
            {
                Move move = precomputedMoves.get(i);
                Piece[][] copy = copyBoard(tempBoard);
                simulateMove(copy, move);

                // Next moves for AI
                List<Move> nextMoves = getAllPossibleMoves(copy, Piece.AI);
                int eval = minimaxAlphaBeta(copy, nextMoves, depth - 1, alpha, beta, true)[0];

                if (eval < minEval) 
                {
                    minEval = eval;
                    bestMoveIndex = i;
                }
                beta = Math.min(beta, minEval);
                if (beta <= alpha) 
                {
                    break;
                }
            }
            return new int[] { minEval, bestMoveIndex };
        }
    }

    /**
     * A multi-factor evaluation of the board:
        - Weighted piece difference
        - Position scoring (center vs walls, early vs endgame)
        - Mobility (number of possible moves)
        - Threat checks
    */
    private int evaluateBoard(Piece[][] tempBoard) 
    {
        int aiCount     = countPieces(Piece.AI, tempBoard);
        int playerCount = countPieces(Piece.PLAYER, tempBoard);
        int totalPieces = aiCount + playerCount;

        // Weighted piece advantage (heavier in endgame)
        double pieceWeight = (12 - totalPieces) / 6.0; 
        int baseScore = (int) ((aiCount - playerCount) * 100 * pieceWeight);

        // Summaries
        int aiPositionScore = 0,    playerPositionScore = 0;
        int aiMobility      = 0,    playerMobility      = 0;
        int aiThreats       = 0,    playerThreats       = 0;

        // Evaluate each cell
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                Piece piece = tempBoard[row][col];
                if (piece == Piece.AI) 
                {
                    aiPositionScore += evaluatePosition(row, col, totalPieces);
                    aiMobility      += countMoves(tempBoard, row, col);
                    aiThreats       += evaluateThreats(tempBoard, row, col, Piece.AI);
                }
                else if (piece == Piece.PLAYER) 
                {
                    playerPositionScore += evaluatePosition(row, col, totalPieces);
                    playerMobility      += countMoves(tempBoard, row, col);
                    playerThreats       += evaluateThreats(tempBoard, row, col, Piece.PLAYER);
                }
            }
        }

        // Weighted combination
        double positionWeight = (totalPieces > 5) ? 1.0 : 0.6; 
        int positionScore = (int) ((aiPositionScore - playerPositionScore) * positionWeight);

        int mobilityScore = (aiMobility - playerMobility) * 18;
        int threatScore   = (aiThreats - playerThreats)   * 25;

        return baseScore + positionScore + mobilityScore + threatScore;
    }

    /**
     * Evaluates the strategic value of a position based on game phase.
    */
    private int evaluatePosition(int row, int col, int totalPieces) 
    {
        // prefer center control
        if (totalPieces >= 5) 
        {
            int distFromCenter = Math.abs(row - 3) + Math.abs(col - 3);
            return 20 - (distFromCenter * 3);
        }
        return 5; 
    }

    /**
     * Counts possible moves for a piece at a given position.
    */
    private int countMoves(Piece[][] tempBoard, int row, int col) 
    {
        int moves = 0;
        int[][] directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };
        for (int[] dir : directions)
        {
            int newRow = row + dir[0];
            int newCol = col + dir[1];
            if (isValidMove(row, col, newRow, newCol, tempBoard)) 
            {
                moves++;
            }
        }
        return moves;
    }

    /**
     * Evaluates threats for/against a piece (like near-wall or sandwich danger).
    */
    private int evaluateThreats(Piece[][] tempBoard, int row, int col, Piece currentPiece) 
    {
        int threatScore = 0;
        if (isThreatenedByWall(tempBoard, row, col, currentPiece)) 
        {
            threatScore -= 50;
        }
        if (isThreatenedBySandwich(tempBoard, row, col, currentPiece)) 
        {
            threatScore -= 50;
        }
        if (canMakeCaptureNextMove(tempBoard, row, col, currentPiece)) 
        {
            threatScore += 50;
        }
        return threatScore;
    }

    private boolean isThreatenedByWall(Piece[][] tempBoard, int row, int col, Piece currentPiece) 
    {
        Piece opponent = (currentPiece == Piece.AI) ? Piece.PLAYER : Piece.AI;
        
        // If on a wall row/col, see if adjacent cell has an opponent
        if (row == 0 || row == 6 || col == 0 || col == 6) 
        {
            int[][] directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };
            for (int[] dir : directions) 
            {
                int rr = row + dir[0];
                int cc = col + dir[1];
                if (rr >= 0 && rr < 7 && cc >= 0 && cc < 7) 
                {
                    if (tempBoard[rr][cc] == opponent) 
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private boolean isThreatenedBySandwich(Piece[][] tempBoard, int row, int col, Piece currentPiece) 
    {
        Piece opponent = (currentPiece == Piece.AI) ? Piece.PLAYER : Piece.AI;
        
        // Check horizontal sandwich
        if (col > 0 && col < 6) 
        {
            if ((tempBoard[row][col-1] == opponent && tempBoard[row][col+1] == Piece.NONE) ||(tempBoard[row][col+1] == opponent && tempBoard[row][col-1] == Piece.NONE)) 
            {
                return true;
            }
        }
        
        // Check vertical sandwich
        if (row > 0 && row < 6) 
        {
            if ((tempBoard[row-1][col] == opponent && tempBoard[row+1][col] == Piece.NONE)  ||(tempBoard[row+1][col] == opponent && tempBoard[row-1][col] == Piece.NONE)) 
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if a piece can make a capture on its next move.
    */
    private boolean canMakeCaptureNextMove(Piece[][] tempBoard, int row, int col, Piece currentPiece) 
    {
        int[][] directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };
        Piece opponent = (currentPiece == Piece.AI) ? Piece.PLAYER : Piece.AI;
        for (int[] dir : directions) 
        {
            int newRow = row + dir[0];
            int newCol = col + dir[1];
            if (newRow >= 0 && newRow < 7 && newCol >= 0 && newCol < 7 && tempBoard[newRow][newCol] == Piece.NONE) 
            {
                Piece[][] copy = copyBoard(tempBoard);
                copy[newRow][newCol] = currentPiece;
                copy[row][col] = Piece.NONE;
                List<Point> capturePoints = new ArrayList<>();
                checkCaptureWithWall(copy, newRow, newCol, -1, 0, opponent, capturePoints);
                checkCaptureWithWall(copy, newRow, newCol, 1, 0, opponent, capturePoints);
                checkCaptureWithWall(copy, newRow, newCol, 0, -1, opponent, capturePoints);
                checkCaptureWithWall(copy, newRow, newCol, 0, 1, opponent, capturePoints);
                
                checkSandwichCapture(copy, newRow, newCol, 1, 0, capturePoints);
                checkSandwichCapture(copy, newRow, newCol, -1, 0, capturePoints);
                checkSandwichCapture(copy, newRow, newCol, 0, 1, capturePoints);
                checkSandwichCapture(copy, newRow, newCol, 0, -1, capturePoints);

                if (!capturePoints.isEmpty()) 
                {
                    return true;
                }
            }
        }
        return false;
    }

    // ========================== AI LOGIC =========================

    /**
     * Runs the AI move logic in a background thread to prevent UI blocking.
    */
    private void runAIInBackground() 
    {
        System.out.println("Starting AI's background thread...");
        currentState = GameState.AI_FIRST_TURN;

        SwingWorker<Void, Void> worker = new SwingWorker<>() {
            @Override
            protected Void doInBackground() 
            {
                System.out.println("AI is performing its move(s) in doInBackground()...");
                performAIMove();
                return null;
            }
            @Override
            protected void done() 
            {
                System.out.println("AI move complete. Checking game status...");
                if (!isGameOver) 
                {
                    currentState = GameState.PLAYER_FIRST_TURN;
                }
                repaint();
            }
        };
        worker.execute();
    }

    /**
     * Orchestrates the AI's move logic:
        - If AI has >1 piece, it can make up to 2 moves; otherwise 1.
        - The second move cannot be with the same piece used first.
    */
    private void performAIMove() 
    {
        if (isGameOver) return;
        System.out.println("AI is trying to make a move...");

        int aiPieceCount = countPieces(Piece.AI, board);
        int maxMoves = (aiPieceCount > 1) ? 2 : 1;

        Point firstMovePieceLocation = null;

        for (int i = 0; i < maxMoves; i++) 
        {
            if (isGameOver) break;

            // Get all possible moves for AI
            List<Move> possibleMoves = getAllPossibleMoves(board, Piece.AI);
            if (possibleMoves.isEmpty()) {
                System.out.println("AI has no possible moves.");
                break;
            }

            // For the second move, exclude the piece used first
            if (i == 1 && firstMovePieceLocation != null) 
            {
                final Point localFirst = firstMovePieceLocation; 
                possibleMoves.removeIf(move -> move.fromRow == localFirst.x && move.fromCol == localFirst.y);
                
                if (possibleMoves.isEmpty()) 
                {
                    System.out.println("AI has no possible second move for a different piece.");
                    break;
                }
            }

            // Use Minimax with alpha-beta, passing the precomputed move list
            
            int depth = 6;  // adjust for performance vs. skill
            
            int[] result = minimaxAlphaBeta(copyBoard(board), possibleMoves, depth, Integer.MIN_VALUE, Integer.MAX_VALUE,true);
            int bestIndex = result[1];
            System.out.println("bestMoveIndex: " + bestIndex + ", possibleMoves.size(): " + possibleMoves.size());

            // fallback if bestIndex is invalid
            if (bestIndex < 0 || bestIndex >= possibleMoves.size()) 
            {
                if (!possibleMoves.isEmpty()) 
                {
                    bestIndex = 0;
                    System.out.println("[FALLBACK] Choosing the first available move.");
                } 
                else 
                {
                    System.out.println("AI has no possible moves at all.");
                    break;
                }
            }

            Move bestMove = possibleMoves.get(bestIndex);
            System.out.println("AI selected: " + bestMove);

            // Execute the best move on the real board
            board[bestMove.toRow][bestMove.toCol] = Piece.AI;
            board[bestMove.fromRow][bestMove.fromCol] = Piece.NONE;

            if (i == 0) 
            {
                firstMovePieceLocation = new Point(bestMove.toRow, bestMove.toCol);
            }
            lastMovedPiecePosition = new Point(bestMove.toRow, bestMove.toCol);
            moveCount++;

            // Perform capture checks, game-over condition
            checkCaptures();
            checkGameOverCondition();
            repaint();

            if (isGameOver) break;
        }
        
        // If not over, switch back to the player
        if (!isGameOver) 
        {
            System.out.println("AI turn finished, switching to PLAYER.");
            currentState = GameState.PLAYER_FIRST_TURN;
        }
    }

    /**
     * Checks captures on the actual board after a move.
    */
    private void checkCaptures() 
    {
        List<Point> piecesToRemove = new ArrayList<>();
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                if (board[row][col] != Piece.NONE) 
                {
                    Piece currentPiece = board[row][col];
                    Piece opponentPiece = (currentPiece == Piece.PLAYER) ? Piece.AI : Piece.PLAYER;

                    checkCaptureWithWall(board, row, col, -1, 0, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(board, row, col, 1, 0, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(board, row, col, 0, -1, opponentPiece, piecesToRemove);
                    checkCaptureWithWall(board, row, col, 0, 1, opponentPiece, piecesToRemove);

                    checkSandwichCapture(board, row, col, 1, 0, piecesToRemove);
                    checkSandwichCapture(board, row, col, -1, 0, piecesToRemove);
                    checkSandwichCapture(board, row, col, 0, 1, piecesToRemove);
                    checkSandwichCapture(board, row, col, 0, -1, piecesToRemove);
                }
            }
        }
        for (Point p : piecesToRemove) 
        {
            board[p.x][p.y] = Piece.NONE;
        }
        if (!piecesToRemove.isEmpty()) 
        {
            captureOccurred = true;
        }
    }

    /**
     * Checks if the game has ended (0 pieces or 50 moves).
    */
    private void checkGameOverCondition() 
    {
        int aiCount     = countPieces(Piece.AI, board);
        int playerCount = countPieces(Piece.PLAYER, board);

        if (aiCount == 0 && playerCount == 0) 
        {
            gameResult = "Draw: Both sides ran out of pieces!";
            isGameOver = true;
        }
        else if (aiCount == 0) 
        {
            gameResult = "Player wins!";
            isGameOver = true;
        }
        else if (playerCount == 0) 
        {
            gameResult = "AI wins!";
            isGameOver = true;
        }
        else if (moveCount >= 50)
        {
            if (playerCount > aiCount)
            {
                gameResult = "After 50 moves, Player is ahead! Player wins.";
            } 
            else if (playerCount < aiCount) 
            {
                gameResult = "After 50 moves, AI is ahead! AI wins.";
            }
            else 
            {
                gameResult = "After 50 moves, it's a draw!";
            }
            isGameOver = true;
        }
        if (isGameOver) 
        {
            System.out.println("Game Over! " + gameResult);
        }
    }

    // ========================== PLAYER LOGIC & UI =========================

    /**
     * Handles player's clicks on the board.
    */
    private void handlePlayerMove(int row, int col) 
    {
        System.out.println("Player clicked row=" + row + ", col=" + col);

        if (currentState == GameState.PLAYER_FIRST_TURN || currentState == GameState.PLAYER_SECOND_TURN) 
        {

            Piece currentPlayer = Piece.PLAYER;
            int remainingPieces = countPieces(currentPlayer, board);
            int maxMoves = (remainingPieces > 1) ? 2 : 1;

            // If no piece is selected yet, select it
            if (selectedPiecePosition == null) 
            {
                if (board[row][col] == currentPlayer && (lastMovedPiecePosition == null || !lastMovedPiecePosition.equals(new Point(row, col)))) 
                {
                    selectedPiecePosition = new Point(row, col);
                    System.out.println("Player selected piece at (" + row + "," + col + ")");
                    repaint();
                }
            } 
            else 
            {
                // Attempt to move the selected piece
                if (isValidMove(selectedPiecePosition.x, selectedPiecePosition.y, row, col, board)) 
                {
                    System.out.println("Player moving piece from ("  + selectedPiecePosition.x + "," + selectedPiecePosition.y  + ") to (" + row + "," + col + ")");

                    board[row][col] = board[selectedPiecePosition.x][selectedPiecePosition.y];
                    board[selectedPiecePosition.x][selectedPiecePosition.y] = Piece.NONE;

                    lastMovedPiecePosition = new Point(row, col);
                    moveCount++;
                    playerMovesMade++;

                    checkCaptures();
                    checkGameOverCondition();

                    // If the player completed their moves, pass to AI
                    if (playerMovesMade >= maxMoves && !isGameOver) 
                    {
                        System.out.println("Player finished moves. Switching to AI turn...");
                        currentState = GameState.AI_FIRST_TURN;
                        playerMovesMade = 0;
                        runAIInBackground();
                    }
                    selectedPiecePosition = null;
                    repaint();
                }
                else 
                {
                    // Invalid move => deselect
                    System.out.println("Invalid move. Deselecting piece.");
                    selectedPiecePosition = null;
                }
            }
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;

        // Draw board and pieces
        for (int row = 0; row < 7; row++) 
        {
            for (int col = 0; col < 7; col++) 
            {
                int x = col * cellSize;
                int y = row * cellSize;

                // Cell border
                g2d.setColor(Color.GRAY);
                g2d.drawRect(x, y, cellSize, cellSize);

                // Highlight selected piece
                if (selectedPiecePosition != null && row == selectedPiecePosition.x && col == selectedPiecePosition.y) 
                {
                    g2d.setColor(SELECTION_COLOR);
                    g2d.fillRect(x, y, cellSize, cellSize);
                }

                // Highlight last moved piece
                if (lastMovedPiecePosition != null && row == lastMovedPiecePosition.x && col == lastMovedPiecePosition.y) 
                {
                    g2d.setColor(LAST_MOVE_COLOR);
                    g2d.fillRect(x, y, cellSize, cellSize);
                }

                // Highlight valid moves for the selected piece
                if (selectedPiecePosition != null) 
                {
                    List<Point> validMoves = getValidMoves(selectedPiecePosition.x, selectedPiecePosition.y);
                    if (validMoves.contains(new Point(row, col))) 
                    {
                        g2d.setColor(VALID_MOVE_COLOR);
                        g2d.fillRect(x, y, cellSize, cellSize);
                    }
                }

                // Draw the piece
                if (board[row][col] == Piece.AI) 
                {
                    g2d.setColor(Color.RED);
                    g2d.fillPolygon(
                        new int[] { x + cellSize/2, x + 10, x + cellSize - 10 },
                        new int[] { y + 10,         y + cellSize - 10, y + cellSize - 10 },
                        3
                    );
                }
                else if (board[row][col] == Piece.PLAYER) 
                {
                    g2d.setColor(Color.BLUE);
                    g2d.fillOval(x + 10, y + 10, cellSize - 20, cellSize - 20);
                }
            }
        }

        // Side panel info
        int infoX = 750;
        int infoY = 50;
        int lineHeight = 30;

        g2d.setFont(new Font("SansSerif", Font.BOLD, 18));
        g2d.setColor(Color.BLACK);

        // Show turn indicator
        String turnMessage;
        switch (currentState) {
            case AI_FIRST_TURN:
            case AI_SECOND_TURN:
                turnMessage = "Current Turn: AI";
                break;
            case PLAYER_FIRST_TURN:
            case PLAYER_SECOND_TURN:
                turnMessage = "Current Turn: Player";
                break;
            default:
                turnMessage = "";
        }
        g2d.drawString(turnMessage, infoX, infoY);

        infoY += lineHeight;
        g2d.drawString("Move Count: " + moveCount, infoX, infoY);
        infoY += lineHeight;
        g2d.drawString("Player Pieces: " + countPieces(Piece.PLAYER, board), infoX, infoY);
        infoY += lineHeight;
        g2d.drawString("AI Pieces: " + countPieces(Piece.AI, board), infoX, infoY);

        if (isGameOver) 
        {
            infoY += lineHeight;
            g2d.setColor(Color.RED);
            g2d.drawString("Game Over!", infoX, infoY);

            infoY += lineHeight;
            g2d.drawString("Result: " + gameResult, infoX, infoY);
        }
    }

    // MouseListener
    @Override
    public void mouseClicked(MouseEvent e) 
    {
        if (!isGameOver) 
        {
            int col = e.getX() / cellSize;
            int row = e.getY() / cellSize;
            handlePlayerMove(row, col);
        }
        repaint();
    }
    
    @Override public void mousePressed(MouseEvent e) {}
    
    @Override public void mouseReleased(MouseEvent e) {}
    
    @Override public void mouseEntered(MouseEvent e) {}
    
    @Override public void mouseExited(MouseEvent e) {}

    /**
     * Main entry point.
    */
    public static void main(String[] args) 
    {
        JFrame frame = new JFrame("7x7 Board Game");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        BoardGameG2DwithAI board = new BoardGameG2DwithAI(100);
        frame.add(board);
        frame.setSize(1024, 740);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);

        System.out.println("Application started. Window is visible.");
    }
}
