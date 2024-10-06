// /app/api/user/current/route.ts
import { NextRequest, NextResponse } from 'next/server';
import { connectToDatabase } from '@/utils/mongodb';
import { fetchUsername } from '@/utils/fetchUsername';

export async function GET(request: NextRequest) {
    const username = await fetchUsername(request); // Get the username from cookies
    console.log("Fetched username:", username);

    if (!username) {
        return NextResponse.json({ error: 'Username not found' }, { status: 401 });
    }

    const { db } = await connectToDatabase();
    const collectionName = `${username}_waste_records`;
    console.log("Collection name:", collectionName); // Debugging line

    try {
        const collection = db.collection(collectionName);
        const documents = await collection.find({}).toArray();
        return NextResponse.json(documents);
    } catch (error) {
        console.error('Error fetching records:', error);
        return NextResponse.json({ error: 'Failed to fetch records' }, { status: 500 });
    }
}
