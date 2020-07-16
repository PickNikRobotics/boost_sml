#include <QApplication>
#include <QCommandLineParser>
#include <QDebug>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QProcess>
#include <QStringList>

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  QGuiApplication::setApplicationDisplayName("sml_viewer");

  QCommandLineParser commandLineParser;
  QCommandLineOption fileOption(QCoreApplication::tr("file"), QCoreApplication::tr("Path to .dot file"), "file");
  commandLineParser.addOption(fileOption);
  commandLineParser.process(QCoreApplication::arguments());

  if (!commandLineParser.isSet(fileOption))
  {
    qFatal("Missing file argument: --file=/PATH/FILENAME.dot");
    return EXIT_FAILURE;
  }

  QString filename = commandLineParser.value(fileOption);
  QString output_filename = "/tmp/sml_transition_diagram.png";
  auto command = QString("dot") + " -Tpng " + filename + " > " + output_filename;

  if (QProcess::execute("/bin/sh", QStringList() << "-c" << command) < 0)
  {
    qFatal("Running command `%s` failed", command.toStdString().c_str());
    return EXIT_FAILURE;
  }

  QGraphicsScene scene;
  QGraphicsView view(&scene);
  QGraphicsPixmapItem item{ QPixmap(output_filename) };
  scene.addItem(&item);
  view.show();
  return a.exec();
}
