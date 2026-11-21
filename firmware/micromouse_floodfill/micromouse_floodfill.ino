/*
 *  Micromouse — flood-fill maze solver (ESP32)
 *
 *  Clean, simplified rewrite of the 2025 micromouse logic.
 *  - 16x16 maze, goal = the 4 center cells
 *  - 3 IR distance sensors (left, front, right) detect walls
 *  - 2 DC motors with encoders move one cell or turn 90 degrees
 *
 *  Each step: read walls -> update map -> re-flood distances -> move to the
 *  neighbor with the lowest distance. Repeats until the mouse reaches the center.
 */

#define N 16

// ---- Pins (adjust to the PCB) ----
#define IR_LEFT    34
#define IR_FRONT   35
#define IR_RIGHT   32
#define ML_PWM     25
#define ML_DIR     26
#define MR_PWM     27
#define MR_DIR     14
#define ENC_L      18
#define ENC_R      19

#define WALL_THRESHOLD   1800   // raw ADC, calibrate on the maze
#define TICKS_PER_CELL    540   // 18 cm cell
#define TICKS_PER_TURN    210   // 90 degrees in place
#define DRIVE_PWM         150

// Wall bits per cell
enum { NORTH = 1, EAST = 2, SOUTH = 4, WEST = 8 };
const int DX[4] = {0, 1, 0, -1};          // N, E, S, W
const int DY[4] = {1, 0, -1, 0};
const uint8_t WALL_BIT[4] = {NORTH, EAST, SOUTH, WEST};

uint8_t walls[N][N];
uint8_t dist[N][N];
int x = 0, y = 0, heading = 0;              // start at (0,0) facing north

volatile long ticksL = 0, ticksR = 0;
void IRAM_ATTR onEncL() { ticksL++; }
void IRAM_ATTR onEncR() { ticksR++; }

bool isGoal(int cx, int cy) { return (cx == 7 || cx == 8) && (cy == 7 || cy == 8); }
bool inside(int cx, int cy) { return cx >= 0 && cx < N && cy >= 0 && cy < N; }

void setWall(int cx, int cy, int dir) {
  walls[cx][cy] |= WALL_BIT[dir];
  int nx = cx + DX[dir], ny = cy + DY[dir];
  if (inside(nx, ny)) walls[nx][ny] |= WALL_BIT[(dir + 2) % 4];   // same wall seen from the other side
}

// Breadth-first flood from the goal: dist = steps to the center through known walls.
void floodFill() {
  static uint8_t qx[N * N], qy[N * N];
  int head = 0, tail = 0;
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++) {
      dist[i][j] = 255;
      if (isGoal(i, j)) { dist[i][j] = 0; qx[tail] = i; qy[tail++] = j; }
    }
  while (head < tail) {
    int cx = qx[head], cy = qy[head++];
    for (int d = 0; d < 4; d++) {
      if (walls[cx][cy] & WALL_BIT[d]) continue;
      int nx = cx + DX[d], ny = cy + DY[d];
      if (inside(nx, ny) && dist[nx][ny] == 255) {
        dist[nx][ny] = dist[cx][cy] + 1;
        qx[tail] = nx; qy[tail++] = ny;
      }
    }
  }
}

void readWalls() {
  if (analogRead(IR_LEFT)  > WALL_THRESHOLD) setWall(x, y, (heading + 3) % 4);
  if (analogRead(IR_FRONT) > WALL_THRESHOLD) setWall(x, y, heading);
  if (analogRead(IR_RIGHT) > WALL_THRESHOLD) setWall(x, y, (heading + 1) % 4);
}

void drive(int left, int right) {
  digitalWrite(ML_DIR, left >= 0);  analogWrite(ML_PWM, abs(left));
  digitalWrite(MR_DIR, right >= 0); analogWrite(MR_PWM, abs(right));
}

void runTicks(int left, int right, long ticks) {
  ticksL = ticksR = 0;
  drive(left, right);
  while ((ticksL + ticksR) / 2 < ticks) delay(1);
  drive(0, 0);
  delay(50);
}

void turnTo(int dir) {
  int diff = (dir - heading + 4) % 4;
  if (diff == 1) runTicks(DRIVE_PWM, -DRIVE_PWM, TICKS_PER_TURN);
  else if (diff == 3) runTicks(-DRIVE_PWM, DRIVE_PWM, TICKS_PER_TURN);
  else if (diff == 2) runTicks(DRIVE_PWM, -DRIVE_PWM, 2 * TICKS_PER_TURN);
  heading = dir;
}

// Pick the open neighbor closest to the goal; prefer going straight on ties.
int bestDirection() {
  int best = -1, bestDist = 256;
  for (int k = 0; k < 4; k++) {
    int d = (heading + k) % 4;
    if (walls[x][y] & WALL_BIT[d]) continue;
    int nx = x + DX[d], ny = y + DY[d];
    if (inside(nx, ny) && dist[nx][ny] < bestDist) { bestDist = dist[nx][ny]; best = d; }
  }
  return best;
}

void setup() {
  Serial.begin(115200);
  pinMode(ML_DIR, OUTPUT); pinMode(MR_DIR, OUTPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), onEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), onEncR, RISING);

  memset(walls, 0, sizeof(walls));
  for (int i = 0; i < N; i++) {                  // outer border
    walls[i][0] |= SOUTH; walls[i][N - 1] |= NORTH;
    walls[0][i] |= WEST;  walls[N - 1][i] |= EAST;
  }
  delay(2000);                                   // time to place the mouse
}

void loop() {
  if (isGoal(x, y)) { drive(0, 0); Serial.println("Center reached"); while (true) delay(1000); }

  readWalls();
  floodFill();
  int dir = bestDirection();
  if (dir < 0) { drive(0, 0); Serial.println("No path"); while (true) delay(1000); }

  turnTo(dir);
  runTicks(DRIVE_PWM, DRIVE_PWM, TICKS_PER_CELL);
  x += DX[dir];
  y += DY[dir];
  Serial.printf("cell (%d,%d) dist=%d\n", x, y, dist[x][y]);
}
