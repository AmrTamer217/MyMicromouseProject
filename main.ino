// =================== PIN DEFINES ===================
// Right motor (use ENB, IN3, IN4)
#define IN1 14 // backward
#define IN2 13 // forward
#define ENA 26

// Left motor (use ENA, IN1, IN2)
#define ENB 25
#define IN3 33 // backward
#define IN4 27 // forward

// ir sensors input
#define ir_right 15
#define ir_left 21
#define ir_front 5

// buttons
#define green 12
#define yellow 4

// Encoders
#define encA_r 18
#define encB_r 19
#define encA_l 23
#define encB_l 22


int dx[] = {0, 1, 0, -1};
int dy[] = {1, 0, -1, 0};
char dir[] = {'n', 'e', 's', 'w'};
// north, east, south, west

const int N = 16;
int wall[N + 2][N + 2][4] = {};
int dist[N + 2][N + 2] = {};

bool wallRight(){
  if(digitalRead(ir_right) == 0) return 1;
  else return 0;
}
bool wallLeft(){
  if(digitalRead(ir_left) == 0) return 1;
  else return 0;
}
bool wallFront(){
  if(digitalRead(ir_front) == 0) return 1;
  else return 0;
}

void setMotorLeft(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, pwm);
}
void setMotorRight(int pwm) {
  if (pwm > 0) {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, pwm);
}

volatile long count_left = 0;
volatile long count_right = 0;

void IRAM_ATTR encoderLeftISR() {
  count_left++;
}

// Right encoder ISR
void IRAM_ATTR encoderRightISR() {
  count_right++;
}

void resetEncoders() {
  noInterrupts();
  count_left = 0;
  count_right = 0;
  interrupts();
}

// ================= PID PARAMETERS =================
float Kp = 2.5;   // Proportional gain
float Ki = 0;   // Integral gain (often small or zero for sync)
float Kd = 0.2;   // Derivative gain

float integral = 0;
float lastError = 0;
float ticks_error = 0.999;

// ================= PID FUNCTION =================
int pidCorrection(long leftTicks, long rightTicks) {
  float error = (double(leftTicks) * ticks_error)  - rightTicks;

  integral += error;
  float derivative = error - lastError;

  float output = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  return (int)output;  // This is the correction value
}

// ================= MOVE FORWARD =================
void moveForward(float distance = 400, int basePwm = 80) {
  long targetTicks = (long)(27.025 * distance);

  resetEncoders();
  integral = 0;
  lastError = 0;

  while ((abs(count_left) < targetTicks) && (abs(count_right) < targetTicks)) {
    int correction = pidCorrection(count_left, count_right);

    int pwmLeft = basePwm - correction;
    int pwmRight = basePwm + correction;

    // Limit PWM range [0..255]
    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    setMotorLeft(pwmLeft);
    setMotorRight(pwmRight);
  }
  setMotorLeft(0);
  setMotorRight(0);
  delay(300);
}

// ======= 90° LEFT TURN =======
void turnLeft(int pwm = 70) {
  long targetTicks = 2228;
  resetEncoders();

  // Left backward, right forward
  setMotorLeft(-pwm);
  setMotorRight(pwm);

  while ((abs(count_left) < ((double)(targetTicks) * ticks_error)) || (abs(count_right) < targetTicks)) {
    if (abs(count_left) < ((double)(targetTicks) * ticks_error)) {
      setMotorLeft(-pwm);
    } else {
      setMotorLeft(0);
    }

    if (abs(count_right) < targetTicks) {
      setMotorRight(pwm);
    } else {
      setMotorRight(0);
    }
  }

  setMotorLeft(0);
  setMotorRight(0);
  delay(400);
}

void turnRight(int pwm = 70) {
  long targetTicks = 2335;
  resetEncoders();

  // Left backward, right forward
  setMotorLeft(pwm);
  setMotorRight(-pwm);

  while ((abs(count_left) < ((double)(targetTicks) * ticks_error)) || (abs(count_right) < (targetTicks))) {
    if (abs(count_left) < ((double)(targetTicks) * ticks_error)) {
      setMotorLeft(pwm);
    } else {
      setMotorLeft(0);
    }

    if (abs(count_right) < (targetTicks)) {
      setMotorRight(-pwm);
    } else {
      setMotorRight(0);
    }
  }

  setMotorLeft(0);
  setMotorRight(0);
  delay(400);
}

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoder pins
  pinMode(encA_l, INPUT_PULLUP);
  pinMode(encB_l, INPUT_PULLUP);
  pinMode(encA_r, INPUT_PULLUP);
  pinMode(encB_r, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA_l), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA_r), encoderRightISR, CHANGE);

  // sensors
  pinMode(ir_right, INPUT);
  pinMode(ir_left, INPUT);
  pinMode(ir_front, INPUT);

  //buttons
  pinMode(green, INPUT);
  pinMode(yellow, INPUT);

  pinMode(2, OUTPUT);

  for (int i = 1; i <= N; i++) {
    wall[1][i][3] = 1;
    wall[i][1][2] = 1;
    wall[N][i][1] = 1;
    wall[i][N][0] = 1;

    dist[0][i] = 10000;
    dist[i][0] = 10000;
    dist[N + 1][i] = 10000;
    dist[i][N + 1] = 10000;
  }
  update_dist();
}

// ====== CONFIG ======
const int MAXQ = (N+2)*(N+2);    // maximum queue size

// ====== QUEUE ======
struct Node {
    int x, y;
};

struct StaticQueue {
    Node arr[MAXQ];
    int front = 0, back = 0;

    inline bool empty() { return front == back; }
    inline void push(Node n) {
        arr[back] = n;
        back = (back + 1) % MAXQ;
    }
    inline Node pop() {
        Node n = arr[front];
        front = (front + 1) % MAXQ;
        return n;
    }
};

// ====== FLOODFILL ======
void update_dist() {
    StaticQueue q;

    q.push({8, 8}); q.push({8, 9}); q.push({9, 8}); q.push({9, 9});
    static bool vis[N+2][N+2];   // reuse between calls
    memset(vis, 0, sizeof(vis));

    vis[8][8] = vis[8][9] = vis[9][8] = vis[9][9] = 1;

    while (!q.empty()) {
        Node cur = q.pop();
        int x = cur.x, y = cur.y;
        for (int d = 0; d < 4; d++) {
            int xx = x + dx[d];
            int yy = y + dy[d];
            if (vis[xx][yy] || wall[x][y][d]) continue;

            q.push({xx, yy});
            vis[xx][yy] = 1;
            dist[xx][yy] = dist[x][y] + 1;
        }
    }
}

void turn(int a, int b) {
  int cloc = (b - a + 4) % 4;
  int anti = (a - b + 4) % 4;
  if (cloc <= anti) {
    while (a % 4 != b) {
      a++;
      turnRight();
    }
  }
  else {
    while ((a + 4) % 4 != b) {
      a--;
      turnLeft();
    }
  }
}

void algo() {
  int x = 1, y = 1, d = 0;
  while (dist[x][y]) {
    wall[x][y][(d + 3) % 4] = wall[x + dx[(d + 3) % 4]][y + dy[(d + 3) % 4]][(d + 3 + 2) % 4] = wallLeft();
    wall[x][y][(d + 1) % 4] = wall[x + dx[(d + 1) % 4]][y + dy[(d + 1) % 4]][(d + 1 + 2) % 4] = wallRight();
    wall[x][y][d] = wall[x + dx[d]][y + dy[d]][(d + 2) % 4] = wallFront();

    update_dist();

    int b = 0;
    for (int i = 0; i < 4; i++) {
      if (dist[x + dx[i]][y + dy[i]] < dist[x][y] && wall[x][y][i] == 0) {
        b = i;
        break;
      }
    }
    turn(d, b);
    d = b;
    moveForward();
    x += dx[d], y += dy[d];
    while(digitalRead(yellow) == 0) ;
    delay(500);
  }
}

void loop() {
  digitalWrite(2, HIGH);
  if (digitalRead(green) == 1) {
    digitalWrite(2, LOW);
    delay(1000);
    digitalWrite(2, HIGH);
    algo();
  }
  delay(100);
}
