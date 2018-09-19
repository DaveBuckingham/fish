import sys
from imucapture import gui

def main(args=None):
    if args is None:
        args = sys.argv[1:]

    gui.main()

if __name__ == "__main__":
    main()
