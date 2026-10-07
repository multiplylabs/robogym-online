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
                    {'type':'command','forward':.4,'lateral':0,'turn':6,'speed_limit':.3,'movement_profile':'exertion'},
                    {'type':'command','forward':.8,'lateral':0,'turn':0,'speed_limit':None},
                ]:
                    import json
                    yield json.dumps(message)
                    observed.append((stream.command,stream.speed,stream._browser_exertion_movement))
        observed=[]
        await _pump(Socket(),stream)
        self.assertEqual([row[1] for row in observed],[.18,.18,.3,.5])
        self.assertEqual(observed[2][0],(.4,0,6))
        self.assertEqual([row[2] for row in observed],[True,True,True,False])
        independent=Stream()
        _apply_walk_speed(independent,'stealth')
        self.assertEqual(independent.speed,.8)
