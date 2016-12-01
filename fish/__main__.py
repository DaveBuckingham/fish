import sys
from fish import am_gui

def main(args=None):
    if args is None:
        args = sys.argv[1:]

    am_gui.main()

if __name__ == "__main__":
    main()
