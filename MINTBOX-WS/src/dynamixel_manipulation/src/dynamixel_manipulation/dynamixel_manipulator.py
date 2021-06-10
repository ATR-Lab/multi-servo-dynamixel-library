#! /usr/bin/python3

class DynamixelManipulator:
  """Generic class representing a Dynamixel manipulator"""
  def __init__(self, x):
    self.running = False