"""Local-only launcher for car_console.html (Python standard library only)."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import shutil
import subprocess

DIRECTORY = Path(__file__).resolve().parent


class ConsoleHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        # Avoid keeping an old control script after a tool update.
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


def create_server(port=8123):
    handler = partial(ConsoleHandler, directory=str(DIRECTORY))
    try:
        return ThreadingHTTPServer(('127.0.0.1', port), handler)
    except OSError:
        if port == 0:
            raise
        # Never open an unrelated server that happens to own the preferred port.
        return ThreadingHTTPServer(('127.0.0.1', 0), handler)


def find_browser():
    # Finish every Chrome lookup before considering an Edge fallback.
    for name, relative in (('chrome', 'Google/Chrome/Application/chrome.exe'),
                           ('msedge', 'Microsoft/Edge/Application/msedge.exe')):
        executable = shutil.which(name)
        if executable:
            return executable
        for variable in ('ProgramFiles(x86)', 'ProgramFiles', 'LOCALAPPDATA'):
            base = os.environ.get(variable)
            if base:
                candidate = Path(base) / relative
                if candidate.is_file():
                    return str(candidate)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8123)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('port must be between 0 and 65535')
    if not (DIRECTORY / 'car_console.html').is_file():
        parser.error('car_console.html is missing; keep the launcher beside it')
    with create_server(args.port) as server:
        url = f'http://127.0.0.1:{server.server_port}/car_console.html'
        print(f'Console ready: {url}', flush=True)
        print('Keep this window open. Press Ctrl+C to stop the web server.', flush=True)
        print('Stop and disconnect in the page BEFORE closing it or this window.', flush=True)
        if not args.no_browser:
            browser = find_browser()
            if browser:
                try:
                    subprocess.Popen([browser, url])
                except OSError as error:
                    print(f'Browser launch failed: {error}. Open the URL in Edge/Chrome.', flush=True)
            else:
                print('Edge/Chrome not found. Open the URL manually in desktop Edge/Chrome.', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
