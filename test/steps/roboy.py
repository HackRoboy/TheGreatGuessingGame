from behave import given, when, then
from nose.tools import eq_

@when(u'Roboy is started')
def start(context):
    context.roboy.start()

@then(u'Roboy says')
def says(context):
    eq_(context.text, roboy_speaks(context))

@when(u'the Roboy welcomed everybody')
def step_impl(context):
    context.roboy.start()
    while 'Welcome' not in context.roboy.read():
        pass

@then(u'Roboy asks for first Group name')
def step_impl(context):
    raise NotImplementedError(u'STEP: Then Roboy asks for first Group name')

@given(u'Roboy asks for first Group Name')
def asked_for_first_group(context):
    context.roboy.start()
    while 'Group 1, please' not in roboy_speaks(context):
        pass

@when(u'I say')
def say(context):
    context.roboy.write(context.text)


def roboy_speaks(context):
    return "\n".join([action['speak'] for action in context.roboy.read()])
