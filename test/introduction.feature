Feature: Introduction

  Scenario: Roboy Welcomes Groups
    When Roboy is started
    Then Roboy says
      """
      Welcome to Roboy Guessing Game.
      """

  @skip
  Scenario: Roboy Welcomes Groups
    When the Roboy welcomed the groups
    Then Roboy asks for first Group name

  @skip
  Scenario:
    Given Roboy asks for first Group Name
    When I say
      """
      Number One
      """
    Then Roboy says
      """
      Welcome Team Number One
      """
