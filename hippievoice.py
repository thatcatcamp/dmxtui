#!/usr/bin/env python3
"""
hippievoice.py — plays a random voice line when fakelight enters TRACKING state.

Usage:
    python3 hippievoice.py [--audio-dir ./audio] [--db path/to/fakelight.db]

Audio files: WAV, 16kHz mono recommended. Place in ./audio/ (or --audio-dir).
If no --db is given, the newest fakelight_*.db in the current directory is used.
"""

import argparse
import glob
import os
import random
import sqlite3
import subprocess
import sys
import time

POLL_INTERVAL_S  = 0.1   # how often to check the DB
RETRIGGER_MIN_S  = 15.0  # don't speak again this soon after the last line
TRACKING_STATE   = 'tracking'


def find_latest_db(pattern='fakelight_*.db'):
    dbs = sorted(glob.glob(pattern))
    if not dbs:
        return None
    return dbs[-1]


def get_latest_state(conn):
    """Return the most recent state string from state_log, or None."""
    try:
        cur = conn.execute(
            "SELECT state FROM state_log ORDER BY ts DESC LIMIT 1"
        )
        row = cur.fetchone()
        return row[0] if row else None
    except sqlite3.OperationalError:
        return None


def pick_audio_file(audio_dir):
    wavs = glob.glob(os.path.join(audio_dir, '*.wav'))
    if not wavs:
        return None
    return random.choice(wavs)


def play(path):
    """Fire-and-forget aplay. Returns the Popen handle."""
    return subprocess.Popen(
        ['aplay', '-q', path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main():
    parser = argparse.ArgumentParser(description='FakeLight hippy voice player')
    parser.add_argument('--audio-dir', default='audio',
                        help='Directory containing .wav files (default: ./audio)')
    parser.add_argument('--db', default=None,
                        help='Path to fakelight DB (default: newest fakelight_*.db)')
    args = parser.parse_args()

    db_path = args.db or find_latest_db()
    if not db_path:
        sys.exit('No fakelight DB found. Start fakelight.py first or pass --db.')

    if not os.path.isdir(args.audio_dir):
        sys.exit(f'Audio dir not found: {args.audio_dir}  (create it and add .wav files)')

    print(f'Watching: {db_path}')
    print(f'Audio:    {args.audio_dir}')

    conn = sqlite3.connect(db_path, check_same_thread=False)

    last_state      = None
    last_played_at  = 0.0
    current_proc    = None

    while True:
        # Re-find DB if fakelight restarted and wrote a new one
        if not args.db:
            newest = find_latest_db()
            if newest and newest != db_path:
                print(f'New DB detected: {newest}')
                conn.close()
                db_path = newest
                conn = sqlite3.connect(db_path, check_same_thread=False)
                last_state = None

        state = get_latest_state(conn)

        if state == TRACKING_STATE and last_state != TRACKING_STATE:
            now = time.time()
            already_playing = current_proc and current_proc.poll() is None
            cooled_down     = (now - last_played_at) >= RETRIGGER_MIN_S

            if not already_playing and cooled_down:
                wav = pick_audio_file(args.audio_dir)
                if wav:
                    print(f'TRACKING → playing {os.path.basename(wav)}')
                    current_proc  = play(wav)
                    last_played_at = now
                else:
                    print('TRACKING detected but no .wav files in audio dir')

        last_state = state
        time.sleep(POLL_INTERVAL_S)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
