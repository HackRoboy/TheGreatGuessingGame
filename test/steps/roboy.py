from behave import given, when, then
from nose.tools import eq_

@when(u'Roboy is started')
def step_impl(context):
    context.roboy.start()

@then(u'Roboy says')
def step_impl(context):
    eq_(context.text, context.roboy.read())

@when(u'the Roboy welcomed the groups')
def step_impl(context):
    raise NotImplementedError(u'STEP: When the Roboy welcomed the groups')

@then(u'Roboy asks for first Group name')
def step_impl(context):
    raise NotImplementedError(u'STEP: Then Roboy asks for first Group name')

@given(u'Roboy asks for first Group Name')
def step_impl(context):
    raise NotImplementedError(u'STEP: Given Roboy asks for first Group Name')

@when(u'I say')
def step_impl(context):
    raise NotImplementedError(u'STEP: When I say')
