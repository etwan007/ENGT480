"""Keep desktop shortcut results visible in their terminal window."""
from pi_sync import main

if __name__ == '__main__':
    result = main()
    try:
        input('\nPress Enter to close this window. ')
    except (EOFError, KeyboardInterrupt):
        pass
    raise SystemExit(result)
