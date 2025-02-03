# BoardGame Using 2D Graphics Player vs AI

This project is a **strategy game on a 7x7 grid** with an **AI opponent**. The AI utilizes the **Minimax algorithm with Alpha-Beta pruning** to optimize its moves.

## Features
-  Play against an AI opponent
-  Highlights valid moves
-  Capture mechanics
-  GUI with Java Swing
-  Game-over condition checks

## Installation
Follow these steps to run the project:

```sh
git clone https://github.com/YOUR_USERNAME/BoardGameG2DwithAI.git
cd BoardGameG2DwithAI
javac BoardGameG2DwithAI.java
java BoardGameG2DwithAI
```

## The Rules of the Game
 The board size is 7*7. Triangle symbols are the pieces of Player 1 which is the AI-based player. 
 Circle symbols are the pieces of Player 2 which is the human player. 
 Every player has four pieces, and the game starts with the board configuration, as shown in figure. 
 When the game starts, Player 1 moves first.

  ![image](https://github.com/user-attachments/assets/21ef7269-f22b-469b-a184-d3dd48a58181)

**The Rules of the Moves**

 The pieces can move in both horizontal and vertical directions. Diagonal moves are not 
allowed. 
If the player has more than one piece, the player should make two subsequent moves 
with different pieces. 
If the player has only one piece, the player should make only a single move.

**Capturing Pieces**

 If the player’s single piece or group of pieces are between the wall and the opponent 
piece, they are captured (example scenarios are shown in figures). 
If the player’s single piece or group of pieces are between two opponent pieces, it is captured. 
If both player’s pieces are between two opponent pieces, all of these pieces are captured.

![image](https://github.com/user-attachments/assets/a9502bd3-682e-4617-870b-3c4751f1b645)

![image](https://github.com/user-attachments/assets/e6194d31-f56e-46cc-98c0-eed04a658dab)

![image](https://github.com/user-attachments/assets/e65bab0d-ff93-4e72-833b-5d1a01c0dab3)

![image](https://github.com/user-attachments/assets/270343f5-b086-46d2-b514-dac22aba59c9)

![image](https://github.com/user-attachments/assets/b6cd21f4-69bc-4549-8b39-97c08d5297fd)



**Game End**
 
 When both players do not have any pieces: It is Draw.
 
 When both players have only a single piece: It is Draw.
 
 When the player has some pieces, but the opponent player does not have any pieces: The player wins.
 
 When the player has no pieces, but the opponent player has some pieces: The player losses.
 
 
 *After 50 moves in total:*
 
   If both players have same number of pieces: It is Draw. 
 
   If the player has more pieces: The player wins.
 
   Else: The player losses

## MIT License
This project is licensed under the **MIT License**. See the `LICENSE` file for more details.

