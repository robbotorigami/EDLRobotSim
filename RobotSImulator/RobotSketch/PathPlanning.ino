#include "dubins.h"

int addNodeToPath(double q[3], double x, void* user_data) {
#ifdef SIMULATION
  char buffer[1000];
  ofstream myfile;
  myfile.open("dubinspath.csv", ios::app);
  sprintf_s(buffer, "%f,%f,%f,%f\n", q[0], q[1], q[2], x);
  myfile << buffer;
  myfile.close();
#endif
  DEBUG(".");
  if (pathlength >= 98) {
    DEBUG("INVALID PATH\n");
    while (true);
  }
  path[pathlength][0] = q[0];
  path[pathlength][1] = q[1];
  pathlength++;
  return 0;
}

void generatePath(double q1[3], double q2[3]) {
  DubinsPath path;
  dubins_init(q1, q2, MINIMUM_RAD_CURV, &path); //Define the parameters of the path
  pathlength = 0;

  DEBUG("\nCalculating Dubins path");
  dubins_path_sample_many(&path, addNodeToPath, 0.1, NULL); 

  addNodeToPath(q2, 0, NULL);
  ::path[pathlength][0] = q1[0] + 1 * cos(q1[2]);
  ::path[pathlength][1] = q1[0] + 1 * sin(q1[2]);
  PRINT("(path length: %d)", pathlength);
  DEBUG("done\n");
}
int findclosestpoint(float xpos, float ypos) {
  float bestdist = 1E8;
  int index;
  for (int i = 0; i < pathlength; i++) {
    float dist = pow(path[i][0] - xpos, 2) + pow(path[i][1] - ypos, 2);
    if (dist < bestdist) {
      bestdist = dist;
      index = i;
    }
  }
  return index;
}
