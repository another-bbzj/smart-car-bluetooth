import importlib.util
from pathlib import Path
import socket
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('car_console_server', ROOT / 'Tools/car_console_server.py')
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


class ConsoleServerTests(unittest.TestCase):
    def test_chrome_installation_wins_over_edge_on_path(self):
        with tempfile.TemporaryDirectory() as folder:
            chrome = Path(folder) / 'Google/Chrome/Application/chrome.exe'
            chrome.parent.mkdir(parents=True)
            chrome.touch()
            with patch.dict(server_module.os.environ, {'LOCALAPPDATA': folder}, clear=True), \
                    patch.object(server_module.shutil, 'which', side_effect=lambda name: 'edge.exe' if name == 'msedge' else None):
                self.assertEqual(server_module.find_browser(), str(chrome))

    def test_chrome_on_path_wins_when_both_browsers_exist(self):
        with patch.object(server_module.shutil, 'which', side_effect=lambda name: name + '.exe'):
            self.assertEqual(server_module.find_browser(), 'chrome.exe')

    @unittest.skipUnless(sys.platform == 'win32', 'Windows BAT entry point')
    def test_bat_launches_python_from_path_with_spaces_and_chinese(self):
        with tempfile.TemporaryDirectory(prefix='car 调试 ') as folder:
            for name in ('car_console.bat', 'car_console_server.py', 'car_console.html'):
                shutil.copyfile(ROOT / 'Tools' / name, Path(folder) / name)
            result = subprocess.run(
                ['cmd.exe', '/d', '/c', str(Path(folder) / 'car_console.bat'), '--help'],
                cwd=ROOT, capture_output=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(b'--no-browser', result.stdout)

    def test_serves_console_independent_of_working_directory(self):
        server = server_module.create_server(0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            self.assertEqual(server.server_address[0], '127.0.0.1')
            with urlopen(f'http://127.0.0.1:{server.server_port}/car_console.html', timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertEqual(response.read(), (ROOT / 'Tools/car_console.html').read_bytes())
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_busy_port_uses_free_port_without_contacting_other_server(self):
        with socket.socket() as busy:
            busy.bind(('127.0.0.1', 0))
            busy.listen()
            server = server_module.create_server(busy.getsockname()[1])
            try:
                self.assertNotEqual(server.server_port, busy.getsockname()[1])
            finally:
                server.server_close()


if __name__ == '__main__':
    unittest.main()
