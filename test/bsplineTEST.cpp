#define PY_SSIZE_T_CLEAN
#include <python3.8/Python.h>

#include <iostream>
#include <cmath>

#include "../math/bsplinebasis.h"
#include "../math/vector2d.h"

#ifdef __cplusplus
extern "C" {
#endif

std::vector<HybridAStar::Vector2D> generatepath() {
  std::vector<HybridAStar::Vector2D> vtemp;
  HybridAStar::Vector2D temp;
  double x = 0, y = 0;
  double stepsize = 0.8;
  while(x<5) {
    y = 0.5*x*x-2*x+1;
    temp.setX(x);temp.setY(y);
    vtemp.push_back(std::move(temp));
    x += stepsize;
  }
  return vtemp;
}

std::vector<HybridAStar::Vector2D> generateSpline() {
  HybridAStar::bsplinebasis line(generatepath());

  auto num = line.trajSize();
  auto ctrlpoints = num - 3 + 1;  // when the spline order equals 3

  //auto ctrlpoints = num - 2 + 1; // when the spline order equals 2

  std::cout << "trajectory point size:" << num << "\tcontrol point numbers:" << ctrlpoints << std::endl;

  std::vector<HybridAStar::Vector2D> vtemp;
  double t = 0;
  while (t<(ctrlpoints-1)) {
    vtemp.push_back(line.compute(t));
    t += 0.05;
  }
  return vtemp;
}

int pathSize() {
  return generatepath().size();
}

int trajectorySize() {
  HybridAStar::bsplinebasis line(generatepath());

  return line.trajSize();
}

void splineVector(double x[], double y[], int len) {
  auto splinev = generateSpline();
  for(size_t i = 0; i<len ; ++i) {
    x[i] = splinev[i].getX();
    y[i] = splinev[i].getY();
  }
  return;
}

void pathVector(double x[], double y[], int len) {
  auto path = generatepath();
  for (size_t i = 0; i<path.size(); ++i) {
    x[i] = path[i].getX();
    y[i] = path[i].getY();
  }
  return;
}

// static PyObject* _run(PyObject *self, PyObject *args) {
//   auto res = run();
//   long a = 1;
//   return PyLong_FromLong(a);
// }

// static PyMethodDef BSplinePlot[] = {
//     {
//         "run",
//         _run,
//         METH_VARARGS,
//         ""
//     },
//     {NULL, NULL, 0, NULL} // sentinel
// };

// static struct PyModuleDef bsplineTEST = {
//   PyModuleDef_HEAD_INIT,
//   "bsplineTEST",  // name of python module
//   NULL, // module documentation, maybe null
//   -1,
//   BSplinePlot
// };

// PyMODINIT_FUNC PyInit_bsplineTEST(void)
// {
//   PyObject *m;
//   m = PyModule_Create(&bsplineTEST);
//   if (m == NULL)
//     return NULL;
//   printf("init great_module module\n");
//   return m;
// }

#ifdef __cplusplus
}
#endif
// int main(int argc, char **argv) {
//   ::testing::InitGoogleTest(&argc,argv);
//   run();
//   // return RUN_ALL_TESTS();
// }