"""Sessions must isolate users and survive a transient socket interruption."""
import asyncio
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robogym_online.wasd_server import _serve_client


class Stream:
    control_dt = .02
    styles = ('stealth', 'walk')
    def __init__(self, model=None):
        self.model = model or object()
        self.resets = 0
        self.equipment = 'none'
        self.command = 0
        self.style = 'stealth'
    def fork_session(self):
        return Stream(self.model)
    def reset(self):
        self.resets += 1
    def set_style(self, name):
        self.style = name
    def set_equipment(self, name):
        self.equipment = name


class Socket:
    def __init__(self, token):
        self.request = SimpleNamespace(path=f'/?session={token}')
        self.remote_address = token
        self.sent = []
        self.closed = False
        self.messages = asyncio.Queue()
    async def send(self, data):
        self.sent.append(json.loads(data))
    async def close(self, **kwargs):
        self.closed = True
        await self.messages.put(None)


async def pump(socket, stream):
    while True:
        message = await socket.messages.get()
        if message is None:
            return
        for name, value in message.items():
            setattr(stream, name, value)


class SessionTests(unittest.IsolatedAsyncioTestCase):
    async def test_two_users_and_reconnection_preserve_independent_state(self):
        contract = {'joint_names': ['joint'], 'body_names': ['body']}
        shared = Stream()
        registry = {}
        tasks = []
        with patch('robogym_online.wasd_server._pump', pump):
            a, b = Socket('alice'), Socket('bob')
            for socket in (a, b):
                tasks.append(asyncio.create_task(_serve_client(socket, contract, shared, registry)))
            await asyncio.sleep(.01)
            await a.messages.put({'equipment': 'barbell', 'command': 1, 'style': 'walk'})
            await b.messages.put({'equipment': 'dumbbells', 'command': -1})
            await asyncio.sleep(.01)
            sa, sb = (registry['sessions'][name]['stream'] for name in ('alice', 'bob'))
            self.assertIsNot(sa, sb)
            self.assertIs(sa.model, sb.model)
            self.assertFalse(a.closed)
            self.assertFalse(b.closed)
            self.assertEqual((sa.equipment, sb.equipment), ('barbell', 'dumbbells'))
            self.assertEqual((sa.command, sb.command), (1, -1))
            await a.close()
            await tasks[0]
            reconnect = Socket('alice')
            tasks.append(asyncio.create_task(_serve_client(reconnect, contract, shared, registry)))
            await asyncio.sleep(.01)
            self.assertIs(registry['sessions']['alice']['stream'], sa)
            self.assertEqual((sa.equipment, sa.style, sa.resets), ('barbell', 'walk', 1))
            self.assertFalse(b.closed)
            await reconnect.close()
            await b.close()
            await asyncio.gather(*tasks)

    async def test_capacity_reclaims_oldest_idle_session_without_evicting_users(self):
        import time
        now = time.monotonic()
        registry = {'sessions': {str(i): {'stream': Stream(), 'client': None,
                    'seen': now - (32-i), 'lock': asyncio.Lock()} for i in range(32)}}
        socket = Socket('new-user')
        with patch('robogym_online.wasd_server._pump', pump):
            task = asyncio.create_task(_serve_client(socket, {'joint_names': [], 'body_names': []}, Stream(), registry))
            await asyncio.sleep(.01)
            self.assertEqual(len(registry['sessions']), 32)
            self.assertNotIn('0', registry['sessions'])
            self.assertFalse(socket.closed)
            await socket.close()
            await task

    async def test_failed_hello_does_not_leave_session_marked_active(self):
        class FailedSocket(Socket):
            async def send(self, _):
                raise ConnectionError('Disconnected during handshake')
        registry = {}
        socket = FailedSocket('gone')
        with self.assertRaises(ConnectionError):
            await _serve_client(socket, {'joint_names': [], 'body_names': []}, Stream(), registry)
        self.assertIsNone(registry['sessions']['gone']['client'])
