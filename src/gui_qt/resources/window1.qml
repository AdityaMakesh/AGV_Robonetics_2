import QtQuick 2.0
import QtQuick.Window 2.2

Window {
    id: main_window
    width: 800
    height: 800
    title: "BLUEsat OWR"
    visible: true
    minimumHeight: 600
    minimumWidth: 600

    TextInput {
            id: topic
            x: 40
            y: 335
            width: 80
            height: 20
            text: qsTr("/cam0")
            font.pixelSize: 12
        }
    }
