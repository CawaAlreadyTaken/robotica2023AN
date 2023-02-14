#include <vision/custMsg.h>
#include <iostream>

using namespace std;

int main(){
    vision::custMsg msg;
    msg.index = 1;
    msg.x = 2.0;
    msg.y = 3.0;
    msg.z = 4.0;
    cout << msg;
    return 0;
}