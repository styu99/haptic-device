#include "energyGraph.h"
#include "utility.h"
#include <math.h>
#include "chai3d.h"
#include <fstream>

EnergyGraph::EnergyGraph(cCamera* cam, bool lj, int cluster_size){
    camera = cam;

    addLabel(scope_upper, camera);
    addLabel(scope_lower, camera);


    global_min_known = lj;

    scope = new cScope();
    scope->setLocalPos(0, 60);
    camera->m_frontLayer->addChild(scope);
    scope->setSignalEnabled(true, true, false, false);
    scope->setTransparencyLevel(.7);


    global_minimum = getGlobalMinima(cluster_size);

    if (global_minimum != 0 && lj) {
      if (global_minimum > -50) {
        upper_bound = 0;
        lower_bound = global_minimum - .5;
      } else {
        upper_bound = 0 + (global_minimum * .2);
        lower_bound = global_minimum - 3;
      }
      global_min_known = true;
    } else {
      upper_bound = 0;
      lower_bound = static_cast<int>(cluster_size) * -3;
      global_minimum = 0;
      global_min_known = false;
    }

    scope->setRange(lower_bound, upper_bound);
    scope_upper->setText(cStr(upper_bound));
    scope_lower->setText(cStr(lower_bound));

    // Height was guessedsimulation and added manually - there's probably a better way
    // To do this but the scope height is protected
    scope_upper->setLocalPos(cAdd(scope->getLocalPos(), cVector3d(0, 180, 0)));
    scope_lower->setLocalPos(scope->getLocalPos());

}


//read in global minimum by cluster size
double EnergyGraph::getGlobalMinima(int cluster_size) {
  std::string file_path = "../resources/data/";
  std::string file_name = "global_minima.txt";
  std::ifstream infile(file_path + file_name);
  if (!infile) {
    std::cerr << "Could not open \"" + file_name + "\" for reading" << std::endl;
    std::cerr << "Did you move it to \"" + file_path + "\"?" << std::endl;
    exit(1);
  } else if ((cluster_size < 2) || (cluster_size > 150)) {
    std::cout << "WARNING: \"" + file_name +
                "\" doesn't have data for clusters of this size yet."
         << std::endl;
    std::cout << "The graph may not be accurate." << std::endl;
    return 0;
  }

  int cluster_size_file;
  double minimum;
  while (infile >> cluster_size_file >> minimum) {
    if (cluster_size_file == cluster_size) {
      break;
    }
  }
  //std::cout << "Global minimum:" << minimum << std::endl;
  return minimum;
}

void EnergyGraph::updateGraph(cPrecisionClock *clock, double potentialEnergy){
    double currentTime = clock->getCurrentTimeSeconds();

    // rounds current time to the nearest tenth
    double currentTimeRounded = double(int(currentTime * 10 + .5)) / 10;

    // The number fmod() is compared to is the threshold, this adjusts the
    // timescale
    if (fmod(currentTime, currentTimeRounded) <= .01) {
      scope->setSignalValues(potentialEnergy / 2, global_minimum);
    }
    // scale the graph if the minimum isn't known
    if (!global_min_known) {
      if ((potentialEnergy / 2) < global_minimum) {
        global_minimum = (potentialEnergy / 2);

        // Update scope
        double currentTime = clock->getCurrentTimeSeconds();

        // rounds current time to the nearest tenth
        double currentTimeRounded = double(int(currentTime * 10 + .5)) / 10;

        // The number fmod() is compared to is the threshold, this adjusts the
        // timescale
        if (fmod(currentTime, currentTimeRounded) <= .01) {
          scope->setSignalValues(potentialEnergy / 2, global_minimum);
        }
        // scale the graph if the minimum isn't known
        if (!global_min_known) {
          if ((potentialEnergy / 2) < global_minimum) {
            global_minimum = (potentialEnergy / 2);
          }
          if (global_minimum < scope->getRangeMin()) {
            auto new_lower = scope->getRangeMin() - 25;
            auto new_upper = scope->getRangeMax() - 25;
            scope->setRange(new_lower, new_upper);
            scope_upper->setText(cStr(scope->getRangeMax()));
            scope_lower->setText(cStr(scope->getRangeMin()));
          }
        }
      }
    }
}
