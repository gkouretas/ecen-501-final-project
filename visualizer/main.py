import sys

from boat_scene import BoatVisualizerScene

def main():
    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    view = BoatVisualizerScene()

    view.show()
    app.exec_()

if __name__ == "__main__":
    main()