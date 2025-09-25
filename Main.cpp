#include <bits/stdc++.h>
#include "API.h"
using namespace std;
#define ll long long 

ll dx[] = {0, 1, 0, -1};
ll dy[] = {1, 0, -1, 0};
char dir[] = {'n', 'e', 's', 'w'};
// north, east, south, west

const ll N = 16;
vector<bool> wall[N + 2][N + 2] = {};
ll dist[N + 2][N + 2] = {};

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
        // no overflow check here since maze is bounded
    }
    inline Node pop() {
        Node n = arr[front];
        front = (front + 1) % MAXQ;
        return n;
    }
};

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

// ====== FLOODFILL ======
void update_dist() {
    StaticQueue q;

    // start cells
    q.push({8, 8});
    q.push({8, 9});
    q.push({9, 8});
    q.push({9, 9});

    static bool vis[N+2][N+2];   // reuse between calls
    memset(vis, 0, sizeof(vis));

    vis[8][8] = vis[8][9] = vis[9][8] = vis[9][9] = 1;

    API::clearAllText();

    while (!q.empty()) {
        Node cur = q.pop();
        int x = cur.x, y = cur.y;

        API::setText(x - 1, y - 1, to_string(dist[x][y])); // Arduino String

        for (int d = 0; d < 4; d++) {
            int xx = x + dx[d];
            int yy = y + dy[d];
            if (vis[xx][yy] || wall[x][y][d]) continue;

            q.push({xx, yy});
            vis[xx][yy] = 1;
            dist[xx][yy] = dist[x][y] + 1;
        }
    }

    log("finished floodfill ... updating the screen");

    for (int i = N; i >= 1; i--) {
        for (int j = 1; j <= N; j++) {
            cerr << dist[i][j] << ' ';
        }
        cerr << endl;
    }
    log("done update_dist");
}



void turn(ll a, ll b){
    cerr << "turning from " << a << " to " << b << endl;
    ll clock = (b - a + 4) % 4;
    ll anti = (a - b + 4) % 4;
    if(clock <= anti){
        while(a % 4 != b){
            a++;
            API::turnRight();
        }
    }
    else{
        while((a + 4) % 4 != b){
            a--;
            API::turnLeft();
        }
    }
}

/*
the mapping is different (x and y are swapped)
*/

int main(){
    log("start");
    for(int i = 0; i <= N + 1; i++){
        for(int j = 0; j <= N + 1; j++) wall[i][j] = {0, 0, 0, 0};
    }
    for(int i = 1; i <= N; i++){
        for(int j = 1; j <= N; j++){
            API::setText(i - 1, j - 1, to_string(i) + ", " + to_string(j));
        }
    }
    for(int i = 1; i <= N; i++){
        if(wall[1][i][3] == 0)
            API::setWall(0, i - 1, dir[3]);
        wall[1][i][3] = 1;

        if(wall[i][1][2] == 0)
            API::setWall(i - 1, 0, dir[2]);
        wall[i][1][2] = 1;

        if(wall[N][i][1] == 0)
            API::setWall(N - 1, i - 1, dir[1]);
        wall[N][i][1] = 1;

        if(wall[i][N][0] == 0)
            API::setWall(i - 1, N - 1, dir[0]);
        wall[i][N][0] = 1;

        dist[0][i] = 1e18;
        dist[i][0] = 1e18;
        dist[N + 1][i] = 1e18;
        dist[i][N + 1] = 1e18;
    }
    std::cerr << "updating start" << endl;
    
}