//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#include <iostream>
#include <boost/filesystem.hpp>


#include "SearchScenario.h"
#include "tests.h"

int main() {
   // Run the tests:
   covarianceComparisonTests();
   trajectoryTests();
   intersectionTests();
   propogateTests();

    auto scenario_dir = "scenarios";

    if (!boost::filesystem::exists(scenario_dir)) {
        std::cerr << scenario_dir << " does not exist." << std::endl;
    }

    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr(scenario_dir); itr != end_itr; ++itr) {
        if (boost::filesystem::is_directory(itr->status())) {
            // itr->path() is a directory.
        } else {
            auto curr_scenario_path = itr->path().string();
            auto curr_scenario_name = itr->path().leaf().string();

            if (curr_scenario_name[0] == '.') {
                continue;
            }

            std::cout << "Processing '" << curr_scenario_name << "'" << std::endl;

            auto scenario = numptest::SearchScenario::fromFile(curr_scenario_path);

            scenario.execute();
        }
    }

    return 0;
}
