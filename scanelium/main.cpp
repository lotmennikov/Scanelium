#include "scanelium.h"
#include <QApplication>
#include <qstandardpaths.h>
#include <qdir.h>
#include <stdio.h>


int main(int argc, char *argv[])
{
    QStringList qpaths = QApplication::libraryPaths();
    qpaths.append(".");
    qpaths.append("platforms");
    qpaths.append("imageformats");
    QApplication::setLibraryPaths(qpaths);

	QString user_path = "./";//QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/Scanelium/";
	QDir folder = QDir(user_path);
	if (!folder.exists())
		folder.mkdir(user_path);

	freopen((user_path+"log.txt").toStdString().c_str(), "w", stdout);

	std::cout << user_path.toStdString() << endl; 


	Eigen::initParallel();

	QApplication a(argc, argv);
	Scanelium w;
	w.show();
	return a.exec();
}
