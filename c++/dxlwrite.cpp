#include "Dxl.h"
#include <vector>
#include <iostream>

using namespace std;
int main(int argc, char *argv[])
{
  string file = "/dev/ttyUSB";
  vector<string> ports = get_available_ports(file);
  Dxl dxl(ports[0]);
  vector<int> ids = dxl.scan(25);
  int pos = 100;
  dxl.write(9, pos, 100);
  dxl.write(17, -pos, 100);
  return 0;
}
