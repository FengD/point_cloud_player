/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   12,   11,   11, 0x08,
      43,   37,   11,   11, 0x08,
      68,   37,   11,   11, 0x08,
      93,   37,   11,   11, 0x08,
     118,   37,   11,   11, 0x08,
     146,   37,   11,   11, 0x08,
     175,   37,   11,   11, 0x08,
     202,   37,   11,   11, 0x08,
     229,   37,   11,   11, 0x08,
     258,   37,   11,   11, 0x08,
     286,   37,   11,   11, 0x08,
     319,   11,   11,   11, 0x08,
     349,   11,   11,   11, 0x08,
     377,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0act\0trigerMenu(QAction*)\0"
    "value\0xSliderValueChanged(int)\0"
    "ySliderValueChanged(int)\0"
    "zSliderValueChanged(int)\0"
    "rollSliderValueChanged(int)\0"
    "pitchSliderValueChanged(int)\0"
    "yawSliderValueChanged(int)\0"
    "redSliderValueChanged(int)\0"
    "greenSliderValueChanged(int)\0"
    "blueSliderValueChanged(int)\0"
    "pointSizeSliderValueChanged(int)\0"
    "fileGenerationButtonPressed()\0"
    "lidarConnectButtonPressed()\0"
    "lidarDisconnectButtonPressed()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->trigerMenu((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 1: _t->xSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->ySliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->zSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->rollSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->pitchSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->yawSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->redSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->greenSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->blueSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->pointSizeSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->fileGenerationButtonPressed(); break;
        case 12: _t->lidarConnectButtonPressed(); break;
        case 13: _t->lidarDisconnectButtonPressed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
