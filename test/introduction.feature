Feature: Introduction

  Scenario: Roboy Welcomes Groups
    When Roboy is started
    Then Roboy says
      """
      Welcome to the Guessing Game with Roboy
      """

  Scenario: Roboy Welcomes Groups
    When the Roboy welcomed everybody
    Then Roboy says
      """
      Please tell first Group Name
      """

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
