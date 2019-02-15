/****************************************************************************
** Meta object code from reading C++ file 'addlidardialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "addlidardialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'addlidardialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_AddLidarDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      22,   16,   15,   15, 0x08,
      42,   16,   15,   15, 0x08,
      60,   16,   15,   15, 0x08,
      83,   80,   15,   15, 0x08,
     113,   80,   15,   15, 0x08,
     148,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_AddLidarDialog[] = {
    "AddLidarDialog\0\0input\0nameUpdate(QString)\0"
    "ipUpdate(QString)\0portUpdate(QString)\0"
    "id\0modelGroupButtonsClicked(int)\0"
    "returnTypeGroupButtonsClicked(int)\0"
    "selectFileButtonPressed()\0"
};

void AddLidarDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AddLidarDialog *_t = static_cast<AddLidarDialog *>(_o);
        switch (_id) {
        case 0: _t->nameUpdate((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->ipUpdate((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->portUpdate((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->modelGroupButtonsClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->returnTypeGroupButtonsClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->selectFileButtonPressed(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData AddLidarDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject AddLidarDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_AddLidarDialog,
      qt_meta_data_AddLidarDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &AddLidarDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *AddLidarDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *AddLidarDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_AddLidarDialog))
        return static_cast<void*>(const_cast< AddLidarDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int AddLidarDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
