"""Static file server for the operator panel.

Same as `python3 -m http.server`, but tells the browser not to cache, so an
updated panel shows up on a normal reload.
"""

import argparse
import functools
import http.server


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):  # keep the launch output readable
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--directory", required=True)
    args, _ = parser.parse_known_args()
    handler = functools.partial(NoCacheHandler, directory=args.directory)
    http.server.ThreadingHTTPServer((args.bind, args.port), handler).serve_forever()


if __name__ == "__main__":
    main()
