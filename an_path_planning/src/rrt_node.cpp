#include <ros/ros.h>
#include <rrtstar/rrts.hpp>
#include <rrtstar/system_single_integrator.h>

using namespace RRTstar;
using namespace SingleIntegrator;

typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

int main(int argc, char** argv)
{
  planner_t rrts;
  return 0;
}
