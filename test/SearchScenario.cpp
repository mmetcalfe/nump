//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#include "SearchScenario.h"
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <armadillo>
#include <mutex>

#include "nump.h"
#include "shared/utility/drawing/cairo_drawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "shared/utility/math/angle.h"
#include "shared/utility/math/distributions.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;
using nump::RotatedRectangle;
using nump::BipedRobotModel;

arma::arma_rng::seed_type randomSeed() {
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
    arma::arma_rng::set_seed(seed);
    return seed;
}

numptest::SearchScenario::StateType stateTypeFromYaml(const YAML::Node& yaml) {
    if (yaml.as<std::string>() == "Position") {
        return numptest::SearchScenario::StateType::Position;
    } else if (yaml.as<std::string>() == "PositionBearing") {
        return numptest::SearchScenario::StateType::PositionBearing;
    }

    std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid state type '" << yaml.as<std::string>() << "'." << std::endl;
    exit(1);
}

arma::vec2 vec2FromYaml(const YAML::Node& yaml) {
    return {
            yaml[0].as<double>(),
            yaml[1].as<double>(),
    };
}

arma::vec3 vec3FromYaml(const YAML::Node& yaml) {
    return {
            yaml[0].as<double>(),
            yaml[1].as<double>(),
            yaml[2].as<double>(),
    };
}

arma::mat22 mat22FromYaml(const YAML::Node& yaml) {
    return {
            { yaml[0][0].as<double>(), yaml[0][1].as<double>(), },
            { yaml[1][0].as<double>(), yaml[1][1].as<double>(), },
    };
}

arma::mat33 mat33FromYaml(const YAML::Node& yaml) {
    return {
            { yaml[0][0].as<double>(), yaml[0][1].as<double>(), yaml[0][2].as<double>(), },
            { yaml[1][0].as<double>(), yaml[1][1].as<double>(), yaml[1][2].as<double>(), },
            { yaml[2][0].as<double>(), yaml[2][1].as<double>(), yaml[2][2].as<double>(), },
    };
}

Circle shapeFromYaml(const YAML::Node& yaml) {
    if (yaml["type"].as<std::string>() == "CIRCLE") {
        return {
                {yaml["position"][0].as<double>(), yaml["position"][1].as<double>(),},
                yaml["radius"].as<double>(),
        };
    }

    std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid shape type '" << yaml["type"].as<std::string>() << "'." << std::endl;
    exit(1);
}

nump::BipedRobotModel::KickBox kickBoxConfigFromYaml(const YAML::Node& yaml) {
    nump::BipedRobotModel::KickBox kickBoxConfig;

    kickBoxConfig.kickExtent = yaml["kick_extent"].as<double>();
    kickBoxConfig.footWidth = yaml["foot_width"].as<double>();
    kickBoxConfig.footSep = yaml["foot_sep"].as<double>();
    kickBoxConfig.footFrontX = yaml["foot_front_x"].as<double>();

    return kickBoxConfig;
}

numptest::SearchScenario numptest::SearchScenario::fromFile(const std::string &file_path) {

    YAML::Node scenarioYaml = YAML::LoadFile(file_path);

    numptest::SearchScenario scenario;

    // Search settings:
    if (scenarioYaml["seed"].as<std::string>() == "RANDOM") {
        scenario.cfg_.seed = randomSeed();
    } else {
        scenario.cfg_.seed = scenarioYaml["seed"].as<int>();
    }

    scenario.cfg_.searchTrialDuration = scenarioYaml["search_trial_duration"].as<double>();
    scenario.cfg_.searchTimeLimitSeconds = scenarioYaml["search_time_limit_seconds"].as<double>();
    scenario.cfg_.searchTimeLimitAfterFeasible = scenarioYaml["searchTimeLimitAfterFeasible"].as<double>();
    scenario.cfg_.maxTimeWithoutImprovement = scenarioYaml["maxTimeWithoutImprovement"].as<double>();
    scenario.cfg_.timeBetweenIdealSolutionAttempts = scenarioYaml["timeBetweenIdealSolutionAttempts"].as<double>();
    scenario.cfg_.replanInterval = scenarioYaml["replan_interval"].as<double>();
    scenario.cfg_.numSamples = scenarioYaml["num_samples"].as<int>();

    scenario.cfg_.rrtsSearchTimeLimitAfterFeasible = scenarioYaml["rrtsSearchTimeLimitAfterFeasible"].as<double>();
    scenario.cfg_.rrtsMaxTimeWithoutImprovement = scenarioYaml["rrtsMaxTimeWithoutImprovement"].as<double>();
    scenario.cfg_.rrtsTimeBetweenIdealSolutionAttempts = scenarioYaml["rrtsTimeBetweenIdealSolutionAttempts"].as<double>();

    scenario.cfg_.rrbt.appendRejectCovThreshold = scenarioYaml["rrbt_append_reject_cov_threshold"].as<double>();
    scenario.cfg_.rrbt.appendRejectCostThreshold = scenarioYaml["rrbt_append_reject_cost_threshold"].as<double>();
    scenario.cfg_.rrbt.chanceConstraint = scenarioYaml["rrbt_chance_constraint"].as<double>();
    scenario.cfg_.rrbt.propagateTimeStep = scenarioYaml["rrbt_propagate_time_step"].as<double>();

    scenario.cfg_.ballObstacleRadiusFactor = scenarioYaml["ball_obstacle_radius_factor"].as<double>();
    scenario.cfg_.ballObstacleOffsetFactor = scenarioYaml["ball_obstacle_offset_factor"].as<double>();

    // Drawing:
    scenario.cfg_.drawPeriod = scenarioYaml["draw_period"].as<int>();
    int canvasWidth = scenarioYaml["canvas_size"][0].as<int>();
    int canvasHeight = scenarioYaml["canvas_size"][1].as<int>();
    scenario.cfg_.canvasSize = {canvasWidth, canvasHeight};

    // Scenario description:
    scenario.cfg_.stateType = stateTypeFromYaml(scenarioYaml["state_type"]);

    scenario.cfg_.mapSize = vec2FromYaml(scenarioYaml["map_size"]);
    scenario.cfg_.footprintSize = vec2FromYaml(scenarioYaml["footprint_size"]);

    if (scenario.cfg_.stateType == StateType::Position) {
//        scenario.cfg_.initialState = vec2FromYaml(scenarioYaml["initial_state"]);
//        scenario.cfg_.goalState = vec2FromYaml(scenarioYaml["goal_state"]);
//        scenario.cfg_.initialCovariance = mat22FromYaml(scenarioYaml["initial_covariance"]);
    } else if (scenario.cfg_.stateType == StateType::PositionBearing) {
        scenario.cfg_.initialState = vec3FromYaml(scenarioYaml["initial_state"]);
        scenario.cfg_.goalState = vec3FromYaml(scenarioYaml["goal_state"]);
        scenario.cfg_.initialCovariance = mat33FromYaml(scenarioYaml["initial_covariance"]);
    } else {
        std::cout << __FILE__ << ", " << __LINE__ << ", ERROR: Invalid state type '." << std::endl;
        exit(1);
    }

    for (const auto& obsYaml : scenarioYaml["obstacles"]) {
        scenario.cfg_.obstacles.push_back(shapeFromYaml(obsYaml));
    }

    for (const auto& obsYaml : scenarioYaml["measurement_regions"]) {
        scenario.cfg_.measurementRegions.push_back(shapeFromYaml(obsYaml));
    }

    // Load ball approach config:
    scenario.cfg_.ball = shapeFromYaml(scenarioYaml["ball"]);
    scenario.cfg_.targetAngle = scenarioYaml["target_angle"].as<double>();
    scenario.cfg_.targetAngleRange = scenarioYaml["target_angle_range"].as<double>();
    scenario.cfg_.rrbt.minKickProbability = scenarioYaml["min_kick_probability"].as<double>();
    scenario.cfg_.kbConfig = kickBoxConfigFromYaml(scenarioYaml["kickboxes"]);

    return scenario;
}

void appendTrialToFile(
    std::string fname,
    const numptest::SearchScenario::SearchTrialResult& trialResult,
    std::shared_ptr<std::mutex> resultsFileMutex
) {
    std::mutex tmp;
    std::lock_guard<std::mutex> lock(resultsFileMutex ? *resultsFileMutex : tmp);

    std::ofstream fs(fname, std::ios::out | std::ios::app);
    fs << " - { kickSuccess: " << trialResult.kickSuccess << std::endl;
    fs << ", kickFailure: " << trialResult.kickFailure  << std::endl;
    fs << ", initialState: "
       << "[" << trialResult.initialState(0)
       << "," << trialResult.initialState(1)
       << "," << trialResult.initialState(2)
       << "]" << std::endl;
    fs << ", finalState: "
       << "[" << trialResult.finalState(0)
       << "," << trialResult.finalState(1)
       << "," << trialResult.finalState(2)
       << "]" << std::endl;
    fs << ", errorMultiplier: " << trialResult.errorMultiplier  << std::endl;
    fs << ", collisionFailure: " << trialResult.collisionFailure  << std::endl;
    fs << ", timeLimit: " << trialResult.timeLimit  << std::endl;
    fs << ", targetAngleRange: " << trialResult.targetAngleRange  << std::endl;
    fs << ", finishTime: " << trialResult.finishTime << std::endl;
    fs << ", searchTimeLimitAfterFeasible: " << trialResult.searchTimeLimitAfterFeasible << std::endl;
    fs << ", maxTimeWithoutImprovement: " << trialResult.maxTimeWithoutImprovement << std::endl;
    fs << ", timeBetweenIdealSolutionAttempts: " << trialResult.timeBetweenIdealSolutionAttempts << std::endl;
    fs << ", replanInterval: " << trialResult.replanInterval  << std::endl;
    fs << ", searchTimeLimit: " << trialResult.searchTimeLimit  << std::endl;
    fs << ", numReplans: " << trialResult.numReplans << std::endl;
    fs << ", numKickAttempts: " << trialResult.numKickAttempts << std::endl;
    fs << ", numAlmostKicks: " << trialResult.numAlmostKicks << std::endl;
    fs << ", chanceConstraint: " << trialResult.chanceConstraint << std::endl;
    fs << ", ballObstacleRadiusFactor: " << trialResult.ballObstacleRadiusFactor << std::endl;
    fs << ", ballObstacleOffsetFactor: " << trialResult.ballObstacleOffsetFactor << std::endl;
    fs << ", seed: " << trialResult.seed << std::endl;
    fs << "}" << std::endl;
    fs.close();
}

void writeDataFile(std::string fname, const std::vector<double>& values) {
//    std::ofstream fs(fname, std::ios::out | std::ios::app);
    std::ofstream fs(fname, std::ios::out);
    for (int i = 0; i < values.size(); i++) {
        fs << (i+1) << " " << values[i] << std::endl;
    }
    fs << std::endl;
    fs.close();
}

void numptest::SearchScenario::performRRBTSearch(cairo_t* cr, const std::string& scenario_prefix) {
    // Run RRBT:
     int sampleNum = 0;
     int pointNum = 0;
     auto rrbtTree = nump::RRBT::fromSearchScenario(cfg_, cr,
          [&](const nump::RRBT& rrbt, const nump::RRBT::StateT newState, bool extended) {
              // Make printed output wrap into columns:
              if (sampleNum % 50 == 0) {
                  std::cout << std::endl;
              }
              sampleNum++;

              if (!extended) {
                  std::cout << "c" << std::flush;
                  return;
              }

              pointNum++;

              if (pointNum % cfg_.drawPeriod == 0) {
                  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
                  drawRRBT(cr, rrbt);
                  cairo_show_page(cr);
                  std::cout << "d" << std::flush;
                  return;
              } else {
                  std::cout << "." << std::flush;
              }
          });
     std::cout << std::endl << "#" << std::endl;

     std::cout << "Num nodes: " << rrbtTree.graph.nodes.size() << std::endl;
     std::cout << "Num edges: " << rrbtTree.graph.edges.size() << std::endl;
     int beliefNodeCount = 0;
     for (auto& node : rrbtTree.graph.nodes) {
        beliefNodeCount += node->value.beliefNodes.size();
     }
     std::cout << "Num belief nodes: " << beliefNodeCount << std::endl;

     cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
     drawRRBT(cr, rrbtTree);
     cairo_show_page(cr);

     writeDataFile(scenario_prefix + "_iteration_times.dat", rrbtTree.iterationTimes);
}

void numptest::SearchScenario::performRRTsSearch(cairo_t* cr, const std::string& scenario_prefix) {
    // Run RRBT:
     int sampleNum = 0;
     int pointNum = 0;
     auto rrtsTree = nump::SearchTree::fromRRTs(cfg_, cr,
          [&](const nump::SearchTree& rrts, const nump::SearchTree::StateT newState, bool extended) {
              // Make printed output wrap into columns:
              if (sampleNum % 50 == 0) {
                  std::cout << std::endl;
              }
              sampleNum++;

              if (!extended) {
                  std::cout << "c" << std::flush;
                  return;
              }

              pointNum++;

              if (pointNum % cfg_.drawPeriod == 0) {
                  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
                  drawSearchTree(cr, rrts);
                  cairo_show_page(cr);
                  std::cout << "d" << std::flush;
                  return;
              } else {
                  std::cout << "." << std::flush;
              }
          });
     std::cout << std::endl << "#" << std::endl;

    //  std::cout << "Num nodes: " << rrtsTree.graph.nodes.size() << std::endl;
    //  std::cout << "Num edges: " << rrtsTree.graph.edges.size() << std::endl;
    //  int beliefNodeCount = 0;
    //  for (auto& node : rrtsTree.graph.nodes) {
    //     beliefNodeCount += node->value.beliefNodes.size();
    //  }
    //  std::cout << "Num belief nodes: " << beliefNodeCount << std::endl;

     cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
     drawSearchTree(cr, rrtsTree);
     cairo_show_page(cr);

    //  writeDataFile(scenario_prefix + "_iteration_times.dat", rrtsTree.iterationTimes);
}


numptest::SearchScenario::SearchTrialResult numptest::SearchScenario::simulate(cairo_t* cr, //nump::Path<BipedRobotModel::State> nominalPath,
    std::function<nump::Path<BipedRobotModel::State>(BipedRobotModel::State, BipedRobotModel::MotionCov)> replanFunc
) {
    Transform2D robot = cfg_.initialState;
    BipedRobotModel::EKF robotFilter;
    robotFilter.mean = {cfg_.initialState};
    robotFilter.covariance = cfg_.initialCovariance;

    SearchTrialResult trialResult;
    trialResult.seed = cfg_.seed;
    trialResult.kickSuccess = false;
    trialResult.kickFailure = false;
    trialResult.collisionFailure = false;
    trialResult.errorMultiplier = cfg_.rrbt.errorMultiplier;
    trialResult.timeLimit = cfg_.searchTrialDuration;
    trialResult.initialState = cfg_.initialState;
    trialResult.targetAngleRange = cfg_.targetAngleRange;
    trialResult.finalState = cfg_.initialState;
    trialResult.finishTime = cfg_.searchTrialDuration;
    trialResult.searchTimeLimitAfterFeasible = cfg_.searchTimeLimitAfterFeasible;
    trialResult.maxTimeWithoutImprovement = cfg_.maxTimeWithoutImprovement;
    trialResult.timeBetweenIdealSolutionAttempts = cfg_.timeBetweenIdealSolutionAttempts;
    trialResult.replanInterval = cfg_.replanInterval;
    trialResult.searchTimeLimit = cfg_.searchTimeLimitSeconds;
    trialResult.numReplans = 0;
    trialResult.numKickAttempts = 0;
    trialResult.numAlmostKicks = 0;
    trialResult.chanceConstraint = cfg_.rrbt.chanceConstraint;
    trialResult.ballObstacleRadiusFactor = cfg_.ballObstacleRadiusFactor;
    trialResult.ballObstacleOffsetFactor = cfg_.ballObstacleOffsetFactor;

    auto nominalPath = replanFunc(robotFilter.mean, robotFilter.covariance);
    trialResult.numReplans++;

    std::vector<Transform2D> simulationStates = {robot};
    std::vector<Transform2D> replanningStates = {robot};
    std::vector<BipedRobotModel::EKF> simulationFilters = {robotFilter};

    // double replanInterval = cfg_.searchTimeLimitSeconds;
    double replanInterval = cfg_.replanInterval;
    double lastPlanningTime = 0;
    bool isReplanning = true;

    double walkTimeStep = 0.001;

    double timeStep = 0.1;
    double timeLimit = cfg_.searchTrialDuration; // The robot must kick the ball within 5 minutes.
    auto walker = nominalPath.walk(walkTimeStep);
    std::cout << "Initial walker: " << walker.isFinished() << std::endl;
    std::cout << "Initial path: " << nominalPath.segments.size() << std::endl;

    double currentTime = 0;
    bool pathWasReplaced = false;
    for (currentTime = 0; currentTime < timeLimit; currentTime += timeStep) {

        // bool newReplanning = (currentTime - lastPlanningTime) < cfg_.searchTimeLimitSeconds; // TODO: Replace with actual plan time.
        bool newReplanning = (currentTime - lastPlanningTime) < nominalPath.timeSpentPlanning; // TODO: Replace with actual plan time.
        if (isReplanning && !newReplanning) {
            isReplanning = false;
            std::cout << "REPLANNING COMPLETE: t = " << currentTime << ", " << walker.isFinished() << std::endl;
        }

        // if ((!isReplanning && walker.isFinished()) || currentTime - lastPlanningTime > replanInterval) {
        if (!isReplanning && walker.isFinished()) {
            std::cout << "REPLANNING: t = " << currentTime << std::endl;
            lastPlanningTime = currentTime;
            isReplanning = true;
            replanningStates.push_back(robot);
            nominalPath = replanFunc(robotFilter.mean, robotFilter.covariance);
            trialResult.numReplans++;
            pathWasReplaced = true;
            walker = nominalPath.walk(walkTimeStep);
            std::cout << "New walker: " << walker.isFinished() << std::endl;
            std::cout << "New path segments: " << nominalPath.segments.size() << std::endl;
        }

        // Obtain desired state and control for the current time from the path:
        if (!isReplanning) {
            // walker.stepTo(currentTime);
            // std::cout << "step: t = " << currentTime << std::endl;
            walker.stepBy(timeStep);
        }

        // if (walker.currentWalker.t < 0.5) {
        //     double frac = walker.currentWalker.t / walker.currentWalker.finishTime;
        //     cairo_set_line_width(cr, 0.005);
        //     cairo_set_source_rgb(cr, 1, 1-frac, frac);
        //     utility::drawing::drawRobot(cr, robot, 0.07, true);
        //     cairo_stroke(cr);
        // }

        if (walker.isFinished() || pathWasReplaced) {
            // Check whether a valid kick is possible:
            RotatedRectangle estimatedFootprint = {robotFilter.mean.position, cfg_.footprintSize};
            if (nump::BipedRobotModel::canKickBallAtTarget(
                estimatedFootprint,
                cfg_.ball,
                cfg_.kbConfig,
                cfg_.targetAngle,
                cfg_.targetAngleRange)
            ) {
                ++trialResult.numKickAttempts;

                RotatedRectangle realFootprint = {robot, cfg_.footprintSize};
                if (nump::BipedRobotModel::canKickBallAtTarget(
                    realFootprint,
                    cfg_.ball,
                    cfg_.kbConfig,
                    cfg_.targetAngle,
                    cfg_.targetAngleRange)
                ) {
                    trialResult.kickSuccess = true;
                    std::cout << "KICK SUCCESS" << std::endl;
                    break;
                } else {
                    trialResult.kickFailure = true;
                    std::cout << "KICK FAILURE" << std::endl;
                }

                if (nump::BipedRobotModel::canAlmostKickBallAtTarget(
                    realFootprint,
                    cfg_.ball,
                    cfg_.kbConfig,
                    cfg_.targetAngle,
                    cfg_.targetAngleRange)
                ) {
                    std::cout << "ALMOST KICK SUCCESS" << std::endl;
                    // trialResult.kickSuccess = true;
                    ++trialResult.numAlmostKicks;
                } else {
                    // trialResult.kickFailure = true;
                    std::cout << "ALMOST KICK FAILURE" << std::endl;
                }
            }
        }

        Transform2D nominalControl = {0.0,0.0,0.0};
        if (!walker.isFinished() && !isReplanning) {
            // arma::vec2 rawControl = walker.currentControl();
            // Transform2D nominalControl = {nominalControl(0), 0, nominalControl(1)};
            // Transform2D nominalControl = walker.currentOmniControl();
            nominalControl = nump::robotmodel::walkBetween(robotFilter.mean.position, walker.currentSegment->xGoal.position);
            // std::cout << "nominalControl: t = " << nominalControl.t() << std::endl;
        }

        // Make observations to update the state estimate:
//        std::vector<BipedRobotModel::Measurement> measurements;
        std::vector<std::pair<BipedRobotModel::Measurement, BipedRobotModel::MeasurementCov>> measurements;
        for (auto& landmark : cfg_.measurementRegions) {
            BipedRobotModel::Measurement idealMeasurement = BipedRobotModel::observeLandmark({robot}, landmark);
            BipedRobotModel::MeasurementCov measCov = BipedRobotModel::measurementNoiseCovariance(timeStep, {robot}, landmark, cfg_.rrbt.errorMultiplier);
            BipedRobotModel::Measurement actualMeas = utility::math::distributions::randn(idealMeasurement, measCov);
            // if (arma::norm(idealMeasurement) > 10 || arma::norm(actualMeas) > 10) {
            //     std::cout << "landmark: " << landmark.centre << std::endl;
            //     std::cout << "robot: " << robot << std::endl;
            //     std::cout << "idealMeasurement: " << idealMeasurement << std::endl;
            //     std::cout << "actualMeas: " << actualMeas << std::endl;
            //     std::cout << "measCov: " << measCov << std::endl;
            // }
            measurements.push_back({actualMeas, measCov});
        }
        robotFilter.update(timeStep, nominalControl, measurements, cfg_.measurementRegions, cfg_.rrbt.errorMultiplier);

        // Use state estimate and stabiliser to enhance the desired control to correct current error:
        // Transform2D lqrControl =

        // Add noise to obtain actual control:
        Transform2D actualControl = {0.0,0.0,0.0};
        if (!walker.isFinished() && !isReplanning) {
            // BipedRobotModel::MotionCov alpha = { // Trial 2
            //     {0.1, 0.001, 0.001},
            //     {0.001, 0.1, 0.001},
            //     {0.001, 0.001, 0.2}
            // };
            BipedRobotModel::MotionCov alpha = { // Trial 3
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
            };
            alpha *= cfg_.rrbt.errorMultiplier*cfg_.rrbt.errorMultiplier; // Trial 4
            Transform2D controlSquared = nominalControl % nominalControl;
            BipedRobotModel::MotionCov motionCov = arma::diagmat(alpha*controlSquared) + arma::diagmat(Transform2D {1e-8, 1e-8, 1e-8});
            actualControl = utility::math::distributions::randn(1, nominalControl, motionCov);
            // std::cout << "actualControl: t = " << actualControl.t() << std::endl;
        }

        // Apply the actual control to the robot to obtain the new state:
        robot = robot.localToWorld(actualControl * timeStep);

        // Record the new state of the robot:
        simulationStates.push_back(robot);
        simulationFilters.push_back(robotFilter);

        // Check for collision with the ball:
        RotatedRectangle robotFootprint = {robot, cfg_.footprintSize};
        if (utility::math::geometry::intersection::test(cfg_.ball, robotFootprint)) {
            trialResult.collisionFailure = true;
            std::cout << "COLLISION FAILURE" << std::endl;
            break;
        }
    }

    // Record results:
    trialResult.finalState = robot;
    trialResult.finishTime = currentTime;

    // Draw the replanning states:
    cairo_set_line_width(cr, 0.001);
    utility::drawing::cairoSetSourceRGB(cr, {0.4, 0.25, 0.1});
    for (auto& state : replanningStates) {
        utility::drawing::drawRobot(cr, state, 0.04, true);
    }
    cairo_fill(cr);

    // Draw the robot's actual path:
    cairo_set_line_width(cr, 0.001);
    cairo_set_source_rgb(cr, 0.8, 0.5, 0.2);
    int stateNum = 0;
    for (auto& state : simulationStates) {
        if (stateNum++ % 4 != 0) {
            continue;
        }

        utility::drawing::drawRobot(cr, state, 0.02, true);
    }
    cairo_stroke(cr);

    // Draw the robot's filter output:
    arma::vec3 col = {0.2, 0.8, 0.5};
    cairo_set_line_width(cr, 0.001);
    int filterNum = 0;
    for (auto& filter : simulationFilters) {
        // std::cout << "filter.mean.position: " << filter.mean.position.t() << std::endl;
        if (filterNum++ % 4 != 0) {
            continue;
        }

        utility::drawing::cairoSetSourceRGB(cr, col);
        utility::drawing::drawRobot(cr, filter.mean.position, 0.02, true);
        cairo_stroke(cr);
        utility::drawing::cairoSetSourceRGB(cr, col*0.5);
        double drawChanceConstraint = cfg_.isRRBT ? cfg_.rrbt.chanceConstraint : 0.5;
        auto ellipse = utility::math::distributions::confidenceRegion(filter.mean.position.xy(), filter.covariance.submat(0,0,1,1), drawChanceConstraint, 3);
        utility::drawing::drawEllipse(cr, ellipse);
        cairo_stroke(cr);
    }

    // Draw the ball;
    cairo_set_line_width(cr, 0.005);
    cairo_set_source_rgb(cr, 0,0,0);
    utility::drawing::drawCircle(cr, cfg_.ball);
    cairo_stroke(cr);

    if (!cfg_.isRRBT) {
        cairo_set_line_width(cr, 0.005);
        utility::drawing::cairoSetSourceRGB(cr, {0.8, 0.3, 0.3});
        utility::drawing::drawCircle(cr, nump::SearchTree::getBallObstacle(cfg_));
        cairo_stroke(cr);
    }

    if (!cfg_.isRRBT) {
        utility::drawing::cairoSetSourceRGB(cr, {1, 0, 0});
        utility::drawing::drawCircle(cr, {{0,0}, 0.05});
        cairo_fill(cr);
    }

    // Draw the target angle range:
    // double lwNormal = 0.01;
    // cairo_set_source_rgb(cr, 0.5, 1, 0.5);
    // cairo_set_line_width(cr, lwNormal);
    // drawRobot(cr, rrbt.scenario.goalState, r);
    // arma::vec2 target = {std::cos(searchTree.scenario.targetAngle), std::sin(searchTree.scenario.targetAngle)};
    arma::vec2 minTarget = {std::cos(cfg_.targetAngle-0.5*cfg_.targetAngleRange), std::sin(cfg_.targetAngle-0.5*cfg_.targetAngleRange)};
    arma::vec2 maxTarget = {std::cos(cfg_.targetAngle+0.5*cfg_.targetAngleRange), std::sin(cfg_.targetAngle+0.5*cfg_.targetAngleRange)};
    // utility::drawing::drawLine(cr, cfg_.ball.centre, cfg_.ball.centre+target);
    utility::drawing::drawLine(cr, cfg_.ball.centre, cfg_.ball.centre+minTarget);
    utility::drawing::drawLine(cr, cfg_.ball.centre, cfg_.ball.centre+maxTarget);
    cairo_stroke(cr);

    // Draw the initial robot's position and kickboxes:
    RotatedRectangle initialFootprint = {cfg_.initialState, cfg_.footprintSize};
    cairo_set_line_width(cr, 0.01);
    cairo_set_source_rgb(cr, 0.5, 0.5, 0.5);
    utility::drawing::drawRobot(cr, initialFootprint.transform, 0.05, true);
    utility::drawing::drawRotatedRectangle(cr, initialFootprint);
    utility::drawing::drawKickBoxes(cr, initialFootprint.transform, cfg_.kbConfig, cfg_.ball.radius);
    cairo_stroke(cr);

    // // Draw the ideal kicking position:
    // auto kickBoxes = BipedRobotModel::getLocalKickBoxes(cfg_.kbConfig, cfg_.ball.radius);
    // double kickBoxLength = kickBoxes[0].size(0);
    // double approachClearence = kickBoxLength * cfg_.ballObstacleOffsetFactor;
    // BipedRobotModel::State kickPos = BipedRobotModel::getIdealKickingPosition(cfg_.ball, cfg_.kbConfig, cfg_.targetAngle, approachClearence);
    // RotatedRectangle kickFootprint = {kickPos.position, cfg_.footprintSize};
    // cairo_set_line_width(cr, 0.007);
    // cairo_set_source_rgb(cr, 0.5, 1.0, 1.0);
    // utility::drawing::drawRobot(cr, kickFootprint.transform, 0.05, true);
    // utility::drawing::drawRotatedRectangle(cr, kickFootprint);
    // utility::drawing::drawKickBoxes(cr, kickFootprint.transform, cfg_.kbConfig, cfg_.ball.radius);
    // cairo_stroke(cr);

    // Draw the final robot's position and kickboxes:
    RotatedRectangle finalFootprint = {robot, cfg_.footprintSize};
    cairo_set_line_width(cr, 0.005);
    cairo_set_source_rgb(cr, 0.8, 0.5, 0.2);
    if (trialResult.collisionFailure) {
        cairo_set_source_rgb(cr, 0.8, 0, 0);
    } else if (trialResult.kickSuccess) {
        cairo_set_source_rgb(cr, 0, 0.8, 0);
    } else if (trialResult.kickFailure) {
        cairo_set_source_rgb(cr, 0, 0, 0.8);
    }
    utility::drawing::drawRobot(cr, robot, 0.05, true);
    utility::drawing::drawRotatedRectangle(cr, finalFootprint);
    utility::drawing::drawKickBoxes(cr, robot, cfg_.kbConfig, cfg_.ball.radius);
    cairo_stroke(cr);

    return trialResult;
}

void numptest::SearchScenario::simulation(cairo_t* cr) {
    // Simulate the robot following the path:
    cfg_.isRRBT = true;
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
    auto resultRRBt = simulate(cr, [=](auto currentState, auto currentCovariance) {
        auto newScenario = cfg_;
        newScenario.initialState = currentState.position;
        newScenario.initialCovariance = currentCovariance;

        // Obtain a nominal path using a planning algorithm:
        auto rrbtTree = nump::RRBT::fromSearchScenario(newScenario, cr);

        if (!rrbtTree.idealPositionAdded) {
            // Add the goal state to the tree:
            BipedRobotModel::State kickPos = BipedRobotModel::getIdealKickingPosition(cfg_.ball, cfg_.kbConfig, cfg_.targetAngle);
            // arma::vec2 targetVec = utility::math::angle::bearingToUnitVector(cfg_.targetAngle);
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.30,kickPos.position.angle()}});
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.25,kickPos.position.angle()}});
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.20,kickPos.position.angle()}});
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.15,kickPos.position.angle()}});
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.10,kickPos.position.angle()}});
            // rrbtTree.extendRRBT(cr, {Transform2D {kickPos.position.xy() - targetVec*0.05,kickPos.position.angle()}});
            rrbtTree.extendRRBT(cr, kickPos);
        }

        nump::Path<BipedRobotModel::State> newPath = rrbtTree.getSolutionPath();

        // Draw the search tree and nominal path:
        // drawRRBT(cr, rrbtTree);
        cairo_set_source_rgb(cr, 0.2, 0.5, 0.8);
        cairo_set_line_width(cr, 0.001);
        utility::drawing::drawPath(cr, newPath, 0.2, 0.02);
        cairo_stroke(cr);

        return newPath;
    });
    cairo_show_page(cr);
    appendTrialToFile("rrbtTrials.yaml", resultRRBt, resultsFileMutex);

    // Simulate the robot following the path:
    // simulate(cr, rrtsSolutionPath, [=](auto currentState) {
    cfg_.isRRBT = false;
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
    auto resultRRTs = simulate(cr, [=](auto currentState, auto currentCovariance) {
        auto newScenario = cfg_;
        newScenario.initialState = currentState.position;
        newScenario.initialCovariance = currentCovariance;

        // Obtain a nominal path using a planning algorithm:
        auto rrtsTree = nump::SearchTree::fromRRTs(newScenario, cr);
        nump::Path<BipedRobotModel::State> newPath = rrtsTree.getSolutionPath();

        // Draw the search tree and nominal path:
        // drawSearchTree(cr, rrtsTree);
        // cairo_set_source_rgb(cr, 0.2, 0.5, 0.8);
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));
        utility::drawing::cairoSetSourceRGB(cr, col);
        cairo_set_line_width(cr, 0.001);
        utility::drawing::drawPath(cr, newPath, 0.2, 0.02);
        cairo_stroke(cr);

        return newPath;
    });
    cairo_show_page(cr);
    appendTrialToFile("rrtsTrials.yaml", resultRRTs, resultsFileMutex);
}

void numptest::SearchScenario::execute(const std::string& scenario_prefix) {
    // auto originalSeed = cfg_.seed;

    while (true) {

    auto trialSeed = randomSeed();
    // std::cout << "WARNING: SET SEED IS INACCURATE." << std::endl;
    // // arma::arma_rng::set_seed(originalSeed);
    // arma::arma_rng::set_seed(8513135);

    std::stringstream ss;
    ss << "trial-diagrams/" << scenario_prefix << "_" << trialSeed << "_searchTests.pdf";
    std::string fname = ss.str();
    std::cout << "'" << fname << "'" << std::endl;
    // auto ouptut_img_file_name = scenario_prefix + "_searchTests.pdf";
    // cairo_surface_t *surface = cairo_pdf_surface_create(ouptut_img_file_name.c_str(), cfg_.canvasSize(0), cfg_.canvasSize(1));
    cairo_surface_t *surface = cairo_pdf_surface_create(fname.c_str(), cfg_.canvasSize(0), cfg_.canvasSize(1));
    cairo_t *cr = cairo_create(surface);

    double canvasAspect = cfg_.canvasSize(0) / cfg_.canvasSize(1);
    double mapAspect = cfg_.mapSize(0) / cfg_.mapSize(1);

    if (mapAspect > canvasAspect) {
        double drawingScale = cfg_.canvasSize(0) / cfg_.mapSize(0);
        cairo_scale(cr, drawingScale, drawingScale);
    } else {
        double drawingScale = cfg_.canvasSize(1) / cfg_.mapSize(1);
        cairo_scale(cr, drawingScale, drawingScale);
    }

//    double centeredFrac = 0.85;
//    double borderSize = 0.5 * (1 - centeredFrac) / centeredFrac;
//    cairo_scale(cr, centeredFrac, centeredFrac);
//    cairo_translate(cr, borderSize, borderSize);

        arma::vec trialConfRand = arma::vec(arma::randu(11));

        double theta = utility::math::angle::normalizeAngle(trialConfRand(0)*arma::datum::pi*2);
        double ballDist = 0.2 + trialConfRand(1)*0.6;
        double heading = utility::math::angle::normalizeAngle(trialConfRand(2)*arma::datum::pi*2);
        arma::vec2 unitvec = utility::math::angle::bearingToUnitVector(theta);
        arma::vec2 xyPos = cfg_.ball.centre + unitvec * ballDist;
        Transform2D initialState = {xyPos,heading};
        double targetAngleRange = arma::datum::pi/2; // trialConfRand(4) * arma::datum::pi/2;
        // double replanInterval = cfg_.replanInterval; // 5; // 1 + trialConfRand(5) * 19;
        // double chanceConstraint = 0.6 + trialConfRand(6)*0.4;
        double chanceConstraint = 0.7; // trialConfRand(6);
        double ballObstacleOffsetFactor = 0.2; // trialConfRand(7);
        double ballObstacleRadiusFactor = 1.0; // 2*trialConfRand(8);
        // int errorMultiplier = 1;// + (int)(4.999*trialConfRand(9)); // explicit rounding to [1, 5]
        // double errorMultiplier = 0.1 + 3.9*trialConfRand(9); // explicit rounding to [1, 5]
        double errorMultiplier = 0.1 + 1.4*trialConfRand(9); // explicit rounding to [1, 5]
        // bool useSquared = trialConfRand(10) < 0.5;

        cfg_.seed = trialSeed;
        // cfg_.ballObstacleOffsetFactor = ballObstacleOffsetFactor;
        // cfg_.ballObstacleRadiusFactor = ballObstacleRadiusFactor;
        cfg_.initialState = initialState;
        cfg_.targetAngleRange = targetAngleRange;
        cfg_.rrbt.errorMultiplier = errorMultiplier;
        // cfg_.searchTimeLimitSeconds = replanInterval;
        cfg_.rrbt.chanceConstraint = chanceConstraint;
        // if (useSquared) {
        //     cfg_.rrbt.chanceConstraint = chanceConstraint*chanceConstraint;
        // }

        std::cout << "{ " << std::endl;
        std::cout << "initialState: "
           << "[" << cfg_.initialState(0)
           << "," << cfg_.initialState(1)
           << "," << cfg_.initialState(2)
           << "]" << std::endl;
        std::cout << ", errorMultiplier: " << cfg_.rrbt.errorMultiplier  << std::endl;
        std::cout << ", targetAngleRange: " << cfg_.targetAngleRange  << std::endl;
        std::cout << ", replanInterval: " << cfg_.replanInterval  << std::endl;
        std::cout << ", searchTimeLimit: " << cfg_.searchTimeLimitSeconds  << std::endl;
        std::cout << ", chanceConstraint: " << cfg_.rrbt.chanceConstraint << std::endl;
        std::cout << ", seed: " << cfg_.seed << std::endl;
        std::cout << "}," << std::endl;

        simulation(cr);


    // performRRBTSearch(cr, scenario_prefix);
    // performRRTsSearch(cr, scenario_prefix);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    }
}
