#pragma once
#ifndef __SQLITEDATABASE_H__
#define __SQLITEDATABASE_H__

#include <string>
#include <vector>
#include <sqlite3.h>

using namespace std;

class SQLiteDatabase
{
public:
	SQLiteDatabase();
	SQLiteDatabase(char* filename);
	~SQLiteDatabase();

	bool open(char* filename);
	vector<vector<string> > query(char* query);
	void close();

private:
	sqlite3 *database;
};

#endif


