// app/api/user/route.ts
import { NextResponse } from "next/server";
import jwt from 'jsonwebtoken';

const JWT_SECRET = process.env.JWT_SECRET as string;

export async function GET(request: Request) {
    const authCookie = request.headers.get('cookie')?.split('; ').find(c => c.startsWith('auth_token='));
    
    if (!authCookie) {
        return NextResponse.json({ error: 'Not authenticated' }, { status: 401 });
    }

    const token = authCookie.split('=')[1];

    try {
        const decoded = jwt.verify(token, JWT_SECRET) as { username: string };
        
        return NextResponse.json({ username: decoded.username });
    } catch (error) {
        return NextResponse.json({ error: 'Invalid token' }, { status: 403 });
    }
}
