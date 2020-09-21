#define PY_SSIZE_T_CLEAN
#include <python3.8/Python.h>
#include <gtest/gtest.h>

#include <iostream>
#include <cmath>

#include "../math/bsplinebasis.h"
#include "../math/vector2d.h"

std::vector<HybridAStar::Vector2D> generatepath() {
  std::vector<HybridAStar::Vector2D> vtemp;
  HybridAStar::Vector2D temp;
  double x = 0, y = 0;
  double stepsize = 0.1;
  while(x<5) {
    y = 0.3*x+2;
    temp.setX(x);temp.setY(y);
    vtemp.push_back(std::move(temp));
    x += stepsize;
  }
  return vtemp;
}

std::vector<HybridAStar::Vector2D> run() {
  HybridAStar::bsplinebasis line(generatepath());

  auto num = line.trajSize();
  auto ctrlpoints = num - 3 + 2;

  std::vector<HybridAStar::Vector2D> vtemp;
  double t = 0;
  while (t<ctrlpoints) {
    vtemp.push_back(line.compute(t));
    t += 0.01;
  }
  return vtemp;
}


static PyObject* _run(PyObject *self, PyObject *args) {
  auto res = run();
  return PyLong_FromLong(1);
}

static PyMethodDef BSplinePlot[] = {
    {
        "run",
        _run,
        METH_VARARGS,
        ""
    },
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef bsplineTEST = {
  PyModuleDef_HEAD_INIT,
  "bsplineTEST",
  NULL,
  -1,
  BSplinePlot
};

PyMODINIT_FUNC PyInit_bsplineTEST(void)
{
  PyObject *m;
  m = PyModule_Create(&bsplineTEST);
  if (m == NULL)
  return NULL;
    printf("init great_module module\n");
  return m;
}

// int main(int argc, char **argv) {
//   ::testing::InitGoogleTest(&argc,argv);
//   run();
//   // return RUN_ALL_TESTS();
// }