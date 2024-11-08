#include "settings.h"

#include <QDebug>
#include <QDir>
#include <QFile>

Q_LOGGING_CATEGORY(logSettings, "Settings")


// ==========================================================================
QVector<float> teleop::convertQStringToQVector(const QString& str) {
    QString        values_str = str;
    auto           values     = values_str.replace(" ", "").split(",");
    QVector<float> array;
    for (int i = 0; i < values.size(); ++i) {
        array.append(values[i].toFloat());
    }
    return array;
}


QString teleop::convertQVectorToQString(const QVector<float>& array) {
    QString str;
    int     end = array.size() - 1;
    for (int i = 0; i < end; ++i) {
        str += QString::number(array[i]) + ", ";
    }
    return str + QString::number(array[end]);
}


teleop::Mode teleop::convertQStringToMode(const QString& str) {
    if (str == "abs") {
        return teleop::Mode::abs;
    } else if (str == "rel") {
        return teleop::Mode::rel;
    } else if (str == "vel") {
        return teleop::Mode::vel;
    } else {
        qCritical(logSettings)
            << "Fail to convert string" << str << "in Mode enum";
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}


QString teleop::convertModeToQString(const teleop::Mode mode) {
    switch (mode) {
        case teleop::Mode::abs:
            return "abs";
        case teleop::Mode::rel:
            return "rel";
        case teleop::Mode::vel:
            return "vel";
        default:
            qCritical(logSettings) << "Fail to convert Mode enum to string ";
            qCritical(logSettings) << " FATAL ";
            exit(EXIT_FAILURE);
    }
}


teleop::RelativeMode teleop::convertQStringToRelativeMode(const QString& str) {
    if (str == "fix") {
        return teleop::RelativeMode::fix;
    } else if (str == "drg") {
        return teleop::RelativeMode::drg;
    } else if (str == "var") {
        return teleop::RelativeMode::var;
    } else {
        qCritical(logSettings)
            << "Fail to convert string" << str << "in RelativeMode enum";
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}


QString teleop::convertRelativeModeToQString(const teleop::RelativeMode mode) {
    switch (mode) {
        case teleop::RelativeMode::fix:
            return "fix";
        case teleop::RelativeMode::drg:
            return "drg";
        case teleop::RelativeMode::var:
            return "var";
        default:
            qCritical(logSettings)
                << "Fail to convert RelativeMode enum to string ";
            qCritical(logSettings) << " FATAL ";
            exit(EXIT_FAILURE);
    }
}


teleop::Movement teleop::convertQStringToMovement(const QString& str) {
    if (str == "lx") {
        return teleop::Movement::lx;
    } else if (str == "ly") {
        return teleop::Movement::ly;
    } else if (str == "lz") {
        return teleop::Movement::lz;
    } else if (str == "sx") {
        return teleop::Movement::sx;
    } else if (str == "sy") {
        return teleop::Movement::sy;
    } else if (str == "sz") {
        return teleop::Movement::sz;
    } else if (str == "sxy") {
        return teleop::Movement::sxy;
    } else if (str == "sxz") {
        return teleop::Movement::sxz;
    } else if (str == "syx") {
        return teleop::Movement::syx;
    } else if (str == "syz") {
        return teleop::Movement::syz;
    } else if (str == "szx") {
        return teleop::Movement::szx;
    } else if (str == "szy") {
        return teleop::Movement::szy;
    } else {
        qCritical(logSettings)
            << "Fail to convert string" << str << "in Movement enum";
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}


QString teleop::convertMovementToQString(const teleop::Movement Movement) {
    switch (Movement) {
        case teleop::Movement::lx:
            return "lx";
        case teleop::Movement::ly:
            return "ly";
        case teleop::Movement::lz:
            return "lz";
        case teleop::Movement::sx:
            return "sx";
        case teleop::Movement::sy:
            return "sy";
        case teleop::Movement::sz:
            return "sz";
        case teleop::Movement::sxy:
            return "sxy";
        case teleop::Movement::sxz:
            return "sxz";
        case teleop::Movement::syx:
            return "syx";
        case teleop::Movement::syz:
            return "syz";
        case teleop::Movement::szx:
            return "szx";
        case teleop::Movement::szy:
            return "szy";
        default:
            qCritical(logSettings)
                << "Fail to convert Movement enum to string ";
            qCritical(logSettings) << " FATAL ";
            exit(EXIT_FAILURE);
    }
}


teleop::FeedbackType teleop::convertQStringToFeedbackType(const QString& str) {
    if (str == "none") {
        return teleop::FeedbackType::none;
    } else if (str == "sphere") {
        return teleop::FeedbackType::sphere;
    } else if (str == "anchor") {
        return teleop::FeedbackType::anchor;
    } else if (str == "linear") {
        return teleop::FeedbackType::linear;
    } else if (str == "triangle") {
        return teleop::FeedbackType::triangle;
    } else if (str == "opponent") {
        return teleop::FeedbackType::opponent;
    } else {
        qCritical(logSettings)
            << "Fail to convert string" << str << "in FeedbackType enum ";
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}


QString
teleop::convertFeedbackTypeToQString(const teleop::FeedbackType feedback) {
    switch (feedback) {
        case teleop::FeedbackType::none:
            return "none";
        case teleop::FeedbackType::sphere:
            return "sphere";
        case teleop::FeedbackType::anchor:
            return "anchor";
        case teleop::FeedbackType::linear:
            return "linear";
        case teleop::FeedbackType::triangle:
            return "triangle";
        case teleop::FeedbackType::opponent:
            return "opponent";
        default:
            qCritical(logSettings)
                << "Fail to convert FeedbackType enum to string";
            qCritical(logSettings) << "FATAL";
            exit(EXIT_FAILURE);
    }
}


teleop::FilterType teleop::convertQStringToFilterType(const QString& str) {
    if (str == "none") {
        return teleop::FilterType::none;
    } else if (str == "sma") {
        return teleop::FilterType::sma;
    } else if (str == "wma") {
        return teleop::FilterType::wma;
    } else if (str == "smm") {
        return teleop::FilterType::smm;
    } else if (str == "blp") {
        return teleop::FilterType::blp;
    } else if (str == "smmblp") {
        return teleop::FilterType::smmblp;
    } else {
        qCritical(logSettings)
            << "Fail to convert string" << str << "in FilterType enum ";
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}


QString teleop::convertFilterTypeToQString(const teleop::FilterType filter) {
    switch (filter) {
        case teleop::FilterType::none:
            return "none";
        case teleop::FilterType::sma:
            return "sma";
        case teleop::FilterType::wma:
            return "wma";
        case teleop::FilterType::smm:
            return "smm";
        case teleop::FilterType::blp:
            return "blp";
        case teleop::FilterType::smmblp:
            return "smmblp";
        default:
            qCritical(logSettings)
                << "Fail to convert FeedbackType enum to string";
            qCritical(logSettings) << "FATAL";
            exit(EXIT_FAILURE);
    }
}


// ==========================================================================
teleop::SettingsManager& teleop::SettingsManager::getInstance() {
    static SettingsManager instance;
    return instance;
}


teleop::SettingsManager::SettingsManager() {
    // setup Dir
    _path.replace("~", QDir::homePath());
    QDir dir;
    if (!dir.exists(_path)) {
        dir.mkpath(_path);
    }
    // setup file
    QFile file;
    file.setFileName(_path + "settings.conf");
    bool must_init = !file.exists();
    _data =
        SettingsPtr(new QSettings(file.fileName(), QSettings::NativeFormat));
    if (must_init) {
        _initConfigs();
    }
    qInfo(logSettings) << "Configuration file correctly readed:"
                       << file.fileName();
    file.close();
}


const QString& teleop::SettingsManager::getDirectoryPath() const {
    return _path;
}


const teleop::SettingsPtr& teleop::SettingsManager::getSettings() const {
    return _data;
}


bool teleop::SettingsManager::getBool(const QString& key) {
    _checkKey(key);
    return _data->value(key).toBool();
}


float teleop::SettingsManager::getFloat(const QString& key) {
    _checkKey(key);
    return _data->value(key).toFloat();
}


unsigned teleop::SettingsManager::getUnsigned(const QString& key) {
    _checkKey(key);
    return _data->value(key).toFloat();
}


QString teleop::SettingsManager::getQString(const QString& key) {
    _checkKey(key);
    return _data->value(key).toString();
}


QVector<float> teleop::SettingsManager::getQVector(const QString& key) {
    _checkKey(key);
    return convertQStringToQVector(_data->value(key).toString());
}


teleop::Mode teleop::SettingsManager::getMode(const QString& key) {
    _checkKey(key);
    return convertQStringToMode(_data->value(key).toString());
}


teleop::RelativeMode
teleop::SettingsManager::getRelativeMode(const QString& key) {
    _checkKey(key);
    return convertQStringToRelativeMode(_data->value(key).toString());
}


teleop::Movement teleop::SettingsManager::getMovement(const QString& key) {
    _checkKey(key);
    return convertQStringToMovement(_data->value(key).toString());
}


teleop::FeedbackType
teleop::SettingsManager::getFeedbackType(const QString& key) {
    _checkKey(key);
    return convertQStringToFeedbackType(_data->value(key).toString());
}


teleop::FilterType teleop::SettingsManager::getFilterType(const QString& key) {
    _checkKey(key);
    return convertQStringToFilterType(_data->value(key).toString());
}


void teleop::SettingsManager::_checkKey(const QString& key) {
    if (!_data->contains(key)) {
        qCritical(logSettings) << "Key not found in settings:" << key;
        qCritical(logSettings) << "FATAL";
        exit(EXIT_FAILURE);
    }
}

void teleop::SettingsManager::_initConfigs() {
    qInfo(logSettings) << "Initialize config file...";

    _data->setValue("nodes/enable_robot", false);
    _data->setValue("nodes/enable_joystick", false);
    _data->setValue("nodes/enable_logging_slave", false);
    _data->setValue("nodes/enable_logging_filters", false);
    _data->setValue("nodes/enable_logging_wrench", false);
    _data->setValue("nodes/log_size", 100000);

    _data->setValue("task/mode", convertModeToQString(Mode::rel));
    _data->setValue("task/relative_mode",
                    convertRelativeModeToQString(RelativeMode::fix));
    _data->setValue("task/new_pose_threshold", 0.0);
    _data->setValue("task/scaling_factor_start", 1.0);
    _data->setValue("task/fla_T_tcp",
                    convertQVectorToQString({0, 0, 45, 0, 180, 0}));
    _data->setValue("task/wsl_T_ori",
                    convertQVectorToQString({100, 0, 200, 0, 0, 0}));
    _data->setValue("task/ori_T_wma",
                    convertQVectorToQString({0, 0, 0, 90, 90, 0}));
    _data->setValue("task/hip_T_pen",
                    convertQVectorToQString({0, 0, 0, 90, 0, 0}));
    _data->setValue("task/twist_filter_type",
                    convertFilterTypeToQString(FilterType::none));
    _data->setValue("task/feedback_type",
                    convertFeedbackTypeToQString(FeedbackType::none));
    _data->setValue("task/feedback_from_robot", false);
    _data->setValue("task/feedback_max_force", 1.0);
    _data->setValue("task/feedback_stiffness", 0.3);
    _data->setValue("task/feedback_tri_dir_pos", 10);
    _data->setValue("task/feedback_tri_offset", 65);
    _data->setValue("task/feedbakc_tri_len", false);

    _data->setValue("touch/period", 5);
    _data->setValue("touch/enable_feedback", false);

    _data->setValue("meca/period", 40);
    _data->setValue("meca/ip", "192.168.0.100");
    _data->setValue("meca/blending", 100);
    _data->setValue("meca/jointVelocity", 100);
    _data->setValue("meca/jointAcceleration", 100);
    _data->setValue("meca/velocityTimeout", 100);
    _data->setValue("meca/cartesianAcceleration", 50);

    _data->setValue("filters/sma", 20);
    _data->setValue("filters/wma", 20);
    _data->setValue("filters/smm", 19);
    _data->setValue("filters/blp",
                    convertQVectorToQString({0.00361257, 0.00722515, 0.00361257,
                                             1.82292669, -0.83737699}));
};
