# CREATING A LPG-TD PLANNER MODULE FOR PLANSYS2.

## Description
<p align= justify>
This project consist of making a new plugin to the ros2_planning_system repository, where you can find another modules like POPF planner or TDF planner.

LPG_TD is very similar to POPF but give us some advantages because supports more things.

Here you can see a little comparation between PLG-TD and POPF:

| Requirement | LPG-TD | POPF |
| ------------- | ------------- | ------------- |
| :strips  | Yes  | Yes |
| :equality | Yes  | Yes |
| :typing  | Yes  | Yes |
| :universal-preconditions  | Yes  | Yes |
| :durative-actions  | Yes  | Yes |
| :conditional-effects  | No  | No |
| :negative-preconditions  | Yes  | No |
| :disjunctive-preconditions  | Yes  | No |
</p>

More information about this planner can be found in the official web page https://lpg.unibs.it/lpg/

Executable code can be found at http://chronus.ing.unibs.it/download/ in the tar file "lpgtd-linux.gz"

**Now, what are the advantages of creating a new module / plugin for ros2_planning_system instead of use the planner as it is?**

As I said before, in plansys2 we already have 2 modules: POPF and TFD. 
This means from just one repository, we can use differents planners according to preference.

How this planner system can use different planners is not difficult to understad:

This planner gets the domain and the problem from the user, copy then to /tmp/ and calls a specific plan solver (lpg-td, popf..) who gets the /tmp/ files and leaves the plan created in a file (also in /tmp/).

The main system waits for that file to be created, pick the information dumped in, and parse it to show on the terminal.

From different planners, we will have the same output structure.

## STEPS TO USE THIS PLUGGIN:

## Getting LPG-td planner:

Decompress `lpgtd-linux.gz` where you want to have the executable code (in your home directory, in a new folder, etc).

Go to your home directory ($HOME) and write in the terminal the next command:

    echo "export LPGTD=[path to executable code]" >> .bashrc


For example, we decompress the executable file in `$HOME/LPGTD-Plan/` so our command would look like this:

    echo "export LPGTD=$HOME/LPGTD-Plan/lpgtd-linux" >> .bashrc

## Creating our pluing

To import plg-td pluing to plansys2, first you need to have ros2_planning_system implemented in your ros2 workspace.

Clone this repository from https://github.com/IntelligentRoboticsLabs/ros2_planning_system

Add `plansys2_lpgtd_plan_solver` to `ros2_planning_system`.

Install all the necessary dependencies using `rosdep` and compile your workspace with `colcon build`.

## How we can run this planner solver?

To use this module, you need to import some lines in `plansys2_bringup/params/plansys2_params.yaml`:

If you open this file you can see POPF and TFD modules.
We need to import our module following the same method:

- `LPGTD:` 

    `plugin: "plansys2/LPGTDPlanSolver"`

Set the parameter `plan_solver_plugins` to `["LPGTD"]` to use this planner over the other ones.

Remember to compile your workspace again.

To run plansys2, write the following command:

    ros2 launch plansys2_bringup plansys2_bringup_launch_distributed.py model_file:=[path to domain.pddl]

In another terminal run:

    ros2 run plansys2_terminal plansys2_terminal

Now you will see a prompt `>` waiting for your problem.
Set all the instances, predicates and your goal.
Run `get plan` and the solution will appear in your terminal.

----------------------------------------------------------------------

However, you may encounter some problems when using the executable. 

Even though the executable is in the appropiate folder, you might have this problem when running ./lpgtd:

    bash: ./lpgtd: No such file or directory

This happens because you are probably trying to run a 32-bit binary on a 64-bit system that doesn't have 32-bit support installed.

To run the executable you need to install libc6:i386, libncurses5:i386 and libstdc++6:i386:

    sudo apt-get update

    sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386

After typing these lines on a terminal, you should be able to use the executable.

## Contributors and github users

Nuria Díaz: [@Nuriadj](https://github.com/Nuriadj)

Roxana Aanei: [@RoxanaAN](https://github.com/RoxanaAN)

Verónica Tornero: [@Veronica274](https://github.com/Veronica274)

Irene Bandera: [@irenebm](https://github.com/irenebm)