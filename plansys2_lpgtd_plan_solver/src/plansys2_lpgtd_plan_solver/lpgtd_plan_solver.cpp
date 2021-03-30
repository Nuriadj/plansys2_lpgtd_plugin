// Copyright 2021 Winx2: Roxana Aanei, Nuria Diaz, Veronica Tornero and Irene Bandera.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_lpgtd_plan_solver/lpgtd_plan_solver.hpp"

namespace plansys2
{

LPGTDPlanSolver::LPGTDPlanSolver()
{
}

std::optional<Plan>
LPGTDPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  system(
    ("$LPGTD -o /tmp/" + node_namespace + "/domain.pddl -f /tmp/" +
    node_namespace + "/problem.pddl -speed -noout > /tmp/" + node_namespace + "/plan").c_str());

  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;
  bool done = false;

  if (plan_file.is_open()) {
    while ((getline(plan_file, line)) && (done == false)) {
      if (!solution) {                  // if there is no solution yet
        if (line.find("Time: (ACTION)") != std::string::npos) {
          solution = true;
        }
      } else if (!line.empty()) {      // if the line is not empty
        PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");
        size_t colon_dot = line.find(";");

        std::string time = line.substr(1, colon_pos - 1);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 3, colon_dot - colon_bra - 3);

        // change from mayus to minus
        for (int i = 0; i < action.length(); i++) {
          action[i] = tolower(action[i]);
        }

        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.push_back(item);
      }
      else if (line.empty()) {        // if the line is empty
        done = true;
        break;
      }
    }
    plan_file.close();
  }

  if (ret.empty()) {
    return {};
  } else {
    return ret;
  }
}

std::string
LPGTDPlanSolver::check_domain(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  system(
    ("$LPGTD-o /tmp/" + node_namespace + "/check_domain.pddl -f /tmp/" +
    node_namespace + "/check_problem.pddl -speed -noout > /tmp/" + node_namespace + "/check.out").c_str());

  std::ifstream plan_file("/tmp/" + node_namespace + "/check.out");

  std::string result((std::istreambuf_iterator<char>(plan_file)),
    std::istreambuf_iterator<char>());

  return result;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LPGTDPlanSolver, plansys2::PlanSolverBase);
