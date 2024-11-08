#ifndef SETTINGS_H
#define SETTINGS_H

#include <QFile>
#include <QLoggingCategory>
#include <QObject>
#include <QSettings>

Q_DECLARE_LOGGING_CATEGORY(logSettings)


namespace teleop {

// ==========================================================================
using SettingsPtr = QSharedPointer<QSettings>;

// ==========================================================================
enum class Mode { abs, rel, vel };
enum class RelativeMode { fix, drg, var };
enum class Movement { lx, ly, lz, sx, sy, sz, sxy, sxz, syx, syz, szx, szy };
enum class FeedbackType { none, sphere, anchor, linear, triangle, opponent };
enum class FilterType { none, sma, wma, smm, blp, smmblp };

// ==========================================================================
QVector<float> convertQStringToQVector(const QString& str);

QString convertQVectorToQString(const QVector<float>& array);

teleop::Mode convertQStringToMode(const QString& str);

QString convertModeToQString(const teleop::Mode mode);

teleop::RelativeMode convertQStringToRelativeMode(const QString& str);

QString convertRelativeModeToQString(const teleop::RelativeMode mode);

teleop::Movement convertQStringToMovement(const QString& str);

QString convertMovementToQString(const teleop::Movement Movement);

teleop::FeedbackType convertQStringToFeedbackType(const QString& str);

QString convertFeedbackTypeToQString(const teleop::FeedbackType feedback);

teleop::FilterType convertQStringToFilterType(const QString& str);

QString convertFilterTypeToQString(const teleop::FilterType filter);

// ==========================================================================
class SettingsManager {  // singleton
  public:
    static SettingsManager& getInstance();

    const QString&     getDirectoryPath() const;
    const SettingsPtr& getSettings() const;

    bool           getBool(const QString& key);
    float          getFloat(const QString& key);
    unsigned       getUnsigned(const QString& key);
    QString        getQString(const QString& key);
    QVector<float> getQVector(const QString& key);
    Mode           getMode(const QString& key);
    RelativeMode   getRelativeMode(const QString& key);
    Movement       getMovement(const QString& key);
    FeedbackType   getFeedbackType(const QString& key);
    FilterType     getFilterType(const QString& key);


  private:
    SettingsManager();
    SettingsManager(const SettingsManager&) = delete;
    void        operator=(const SettingsManager&) = delete;
    QString     _path                             = "~/.meca_teleoperation/";
    SettingsPtr _data;

    void _checkKey(const QString& key);
    void _initConfigs();
};

}  // namespace teleop


#endif  // SETTINGS_H
