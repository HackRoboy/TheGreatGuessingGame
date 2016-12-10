from support.roboy import Roboy


def before_all(context):
    import os
    import shlex

    assert 'sut' in os.environ
    context.roboy = Roboy(shlex.split(os.environ['sut']))

def after_scenario(context, _):
    context.roboy.stop()
