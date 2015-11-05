//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#include <iostream>
#include <thread>
#include <boost/filesystem.hpp>


#include "SearchScenario.h"
#include "tests.h"

int main() {
    arma::wall_clock timer;
    timer.tic();

    // Run the tests:
    // covarianceComparisonTests();
    // trajectoryTests();
    // intersectionTests();
    // propogateTests();
    // std::cout << "Completed all tests." << std::endl;
    // return 0;

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
            auto curr_scenario_file = itr->path().leaf().string();
            if (curr_scenario_file[0] == '.') {
                continue;
            }

            auto curr_scenario_name = itr->path().stem().string();
            std::cout << "Processing '" << curr_scenario_name << "'" << std::endl;

            auto scenario = numptest::SearchScenario::fromFile(curr_scenario_path);
            // scenario.execute(curr_scenario_name);

            // Start some threads:
            scenario.resultsFileMutex = std::make_shared<std::mutex>();
            auto threadFunc = [=]() mutable {
                scenario.execute(curr_scenario_name);
            };
            std::thread t1(threadFunc);
            std::thread t2(threadFunc);
            std::thread t3(threadFunc);
            t1.join();
            t2.join();
            t3.join();
        }
    }

    std::cout << "Completed all scenarios." << std::endl;

    double seconds = timer.toc();
    std::cout << "Total time: " << seconds << " seconds." << std::endl;

    return 0;
}
