/****************************************************************************
** Meta object code from reading C++ file 'polyhedron_array_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/decomp_ros_utils/src/polyhedron_array_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'polyhedron_array_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay_t {
    QByteArrayData data[8];
    char stringdata0[154];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay_t qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay = {
    {
QT_MOC_LITERAL(0, 0, 43), // "decomp_rviz_plugins::Polyhedr..."
QT_MOC_LITERAL(1, 44, 23), // "updateMeshColorAndAlpha"
QT_MOC_LITERAL(2, 68, 0), // ""
QT_MOC_LITERAL(3, 69, 24), // "updateBoundColorAndAlpha"
QT_MOC_LITERAL(4, 94, 21), // "updateVsColorAndAlpha"
QT_MOC_LITERAL(5, 116, 11), // "updateState"
QT_MOC_LITERAL(6, 128, 11), // "updateScale"
QT_MOC_LITERAL(7, 140, 13) // "updateVsScale"

    },
    "decomp_rviz_plugins::PolyhedronArrayDisplay\0"
    "updateMeshColorAndAlpha\0\0"
    "updateBoundColorAndAlpha\0updateVsColorAndAlpha\0"
    "updateState\0updateScale\0updateVsScale"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_decomp_rviz_plugins__PolyhedronArrayDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    0,   47,    2, 0x08 /* Private */,
       6,    0,   48,    2, 0x08 /* Private */,
       7,    0,   49,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void decomp_rviz_plugins::PolyhedronArrayDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PolyhedronArrayDisplay *_t = static_cast<PolyhedronArrayDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateMeshColorAndAlpha(); break;
        case 1: _t->updateBoundColorAndAlpha(); break;
        case 2: _t->updateVsColorAndAlpha(); break;
        case 3: _t->updateState(); break;
        case 4: _t->updateScale(); break;
        case 5: _t->updateVsScale(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject decomp_rviz_plugins::PolyhedronArrayDisplay::staticMetaObject = {
    { &rviz::MessageFilterDisplay<decomp_ros_msgs::PolyhedronArray>::staticMetaObject, qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay.data,
      qt_meta_data_decomp_rviz_plugins__PolyhedronArrayDisplay,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *decomp_rviz_plugins::PolyhedronArrayDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *decomp_rviz_plugins::PolyhedronArrayDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_decomp_rviz_plugins__PolyhedronArrayDisplay.stringdata0))
        return static_cast<void*>(this);
    return rviz::MessageFilterDisplay<decomp_ros_msgs::PolyhedronArray>::qt_metacast(_clname);
}

int decomp_rviz_plugins::PolyhedronArrayDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::MessageFilterDisplay<decomp_ros_msgs::PolyhedronArray>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
