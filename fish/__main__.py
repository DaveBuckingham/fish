import sys
from fish import ic_gui

def main(args=None):
    if args is None:
        args = sys.argv[1:]

    ic_gui.main()

if __name__ == "__main__":
    main()
