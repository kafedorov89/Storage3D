// SQLite_Connect.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
//#include <sqlite3.h>
#include <iostream>
#include <ctime>
#include "SQLiteDatabase.h"

using namespace std;

//const char* SQL = "CREATE TABLE IF NOT EXISTS foo(a,b,c); INSERT INTO FOO VALUES(1,2,3);";

int _tmain(int argc, _TCHAR* argv[])
{
	SQLiteDatabase *db;
	db = new SQLiteDatabase("Storage3D.sqlite");
	//db->query("CREATE TABLE a (a INTEGER, b INTEGER);");
	db->query("INSERT INTO object VALUES(1, 2);");
	vector<vector<string> > result = db->query("SELECT a, b FROM object;");
	for (vector<vector<string> >::iterator it = result.begin(); it < result.end(); ++it)
	{
		vector<string> row = *it;
		cout << "Values: (A=" << row.at(0) << ", B=" << row.at(1) << ")" << endl;
	}
	db->close();

	/*sqlite3 *db = 0; // хэндл объекта соединение к БД
	char *err = 0;

	// открываем соединение
	if (sqlite3_open("my_cosy_database.dblite", &db))
		fprintf(stderr, "Ошибка открытия/создания БД: %s\n", sqlite3_errmsg(db));
	// выполняем SQL
	else if (sqlite3_exec(db, SQL, 0, 0, &err))
	{
		fprintf(stderr, "Ошибка SQL: %sn", err);
		sqlite3_free(err);
	}
	// закрываем соединение
	sqlite3_close(db);*/
	return 0;
}



