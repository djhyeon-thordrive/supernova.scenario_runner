#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario in which the ego is parked between two vehicles and has to maneuver to start the route.
"""

from __future__ import print_function

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorDestroy,
                                                                      ActorTransformSetter,
                                                                      Idle,
                                                                      KeepVelocity,
                                                                      WaitForever,
                                                                      ChangeAutoPilot,
                                                                      ScenarioTimeout)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, ScenarioTimeoutTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario

from srunner.tools.background_manager import ChangeRoadBehavior


def convert_dict_to_location(actor_dict):
    """
    Convert a JSON string to a Carla.Location
    """
    location = carla.Location(
        x=float(actor_dict['x']),
        y=float(actor_dict['y']),
        z=float(actor_dict['z'])
    )
    return location


def get_value_parameter(config, name, p_type, default):
    if name in config.other_parameters:
        return p_type(config.other_parameters[name]['value'])
    else:
        return default


class ParkingLotExit(BasicScenario):
    """
    This class holds everything required for a scenario in which the ego would be teleported to the parking lane.
    Once the scenario is triggered, the OutsideRouteLanesTest will be deactivated since the ego is out of the driving lane.
    Then blocking vehicles will be generated in front of and behind the parking point.
    The ego need to exit from the parking lane and then merge into the driving lane.
    After the ego is {end_distance} meters away from the parking point, the OutsideRouteLanesTest will be activated and the scenario ends.

    Note 1: For route mode, this shall be the first scenario of the route. The trigger point shall be the first point of the route waypoints.

    Note 2: Make sure there are enough space for spawning blocking vehicles.
    """

    def __init__(self, world, ego_vehicles, config, debug_mode=False, criteria_enable=True,
                 timeout=180):
        """
        Setup all relevant parameters and create scenario
        and instantiate scenario manager
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._tm = CarlaDataProvider.get_client().get_trafficmanager(
            CarlaDataProvider.get_traffic_manager_port())
        self.timeout = timeout

        # Get parking_waypoint based on trigger_point
        self._trigger_position = config.trigger_points[0]
        self._start_position = config.trigger_points[1]
        self._end_position = config.trigger_points[2]


        super().__init__("ParkingLotExit",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        actor_crossing = CarlaDataProvider.request_new_actor(
            '*pedestrian.*', self._start_position, rolename='actor1')
        self.other_actors.append(actor_crossing)


        # Move the ego to its side position
        if not self.route_mode:
            self.ego_vehicles[0].set_transform(self._trigger_position)

    def _create_behavior(self):
        """
        Deactivate OutsideRouteLanesTest, then move ego to the parking point,
        generate blocking vehicles in front of and behind the ego.
        After ego drives away, activate OutsideRouteLanesTest, end scenario.
        """

        sequence = py_trees.composites.Sequence(name="ParkingLotExit")

        
        for actor in self.other_actors:
            sequence.add_child(ActorTransformSetter(actor, self._start_position, True))

        main_behavior = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, name="WalkerMovement")

        idx = 1
        for actor in self.other_actors:
            walker_sequence = py_trees.composites.Sequence(name="WalkerCrossing")
            walker_sequence.add_child(Idle(0.1))
            walker_sequence.add_child(KeepVelocity(
                actor, 5*idx, False, 20, 20))
            walker_sequence.add_child(ActorDestroy(actor, name="DestroyAdversary"))
            walker_sequence.add_child(WaitForever())
            main_behavior.add_child(walker_sequence)
            idx += 1
        sequence.add_child(main_behavior)


        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        """
        Remove all actors and traffic lights upon deletion
        """
        self.remove_all_actors()
