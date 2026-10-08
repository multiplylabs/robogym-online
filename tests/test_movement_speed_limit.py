"""Movement limits reach the planner and remain per-session across style changes."""
import unittest
from robogym_online.wasd_server import _apply_walk_speed, _pump

class SpeedLimitTests(unittest.IsolatedAsyncioTestCase):
    async def test_command_limit_style_change_and_clear(self):
        class Stream:
            style='stealth'
            def set_target_speed(self,speed): self.speed=speed
            def set_command(self,*command): self.command=command
            def set_style(self,style): self.style=style
        stream=Stream()
        class Socket:
            async def __aiter__(self):
                for message in [
                    {'type':'command','forward':0,'lateral':.45,'turn':0,'speed_limit':.18,'movement_profile':'exertion'},
                    {'type':'style','name':'slow_walk'},
                    {'type':'command','forward':.4,'lateral':0,'turn':10,'speed_limit':.5,'movement_profile':'exertion_vertical'},
                    {'type':'command','forward':.8,'lateral':0,'turn':0,'speed_limit':None},
                ]:
                    import json
                    yield json.dumps(message)
                    observed.append((stream.command,stream.speed,stream._browser_exertion_movement,stream._browser_vertical_exertion))
        observed=[]
        await _pump(Socket(),stream)
        self.assertEqual([row[1] for row in observed],[.18,.18,.5,.5])
        self.assertEqual(observed[2][0],(.4,0,10))
        self.assertEqual([row[2] for row in observed],[True,True,True,False])
        self.assertEqual([row[3] for row in observed],[False,False,True,False])
        independent=Stream()
        _apply_walk_speed(independent,'stealth')
        self.assertEqual(independent.speed,.8)

    async def test_loaded_slope_speed_restores_on_clear_and_does_not_replace_other_styles(self):
        class Stream:
            style='slow_walk'
            def set_target_speed(self,speed):self.speed=speed
            def set_command(self,*command):self.command=command
            def set_style(self,style):self.style=style
        stream=Stream()
        observed=[]
        class Socket:
            async def __aiter__(self):
                import json
                for request in [
                    {'type':'command','forward':.8,'lateral':0,'turn':0,'movement_profile':'exertion_slope'},
                    {'type':'style','name':'stealth'},
                    {'type':'style','name':'slow_walk'},
                    {'type':'command','forward':.8,'lateral':0,'turn':0,'movement_profile':None},
                ]:
                    yield json.dumps(request)
                    observed.append((stream.speed,stream._browser_slope_exertion))
        await _pump(Socket(),stream)
        self.assertEqual(observed,[(.8,True),(.8,True),(.8,True),(.5,False)])
