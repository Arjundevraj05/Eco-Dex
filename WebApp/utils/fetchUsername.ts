export const fetchUsername = async () => {
    const response = await fetch('/api/user', { credentials: 'include' });
    if (!response.ok) {
        return null;
    }

    const data = await response.json();
    const username = data.username;

    if (!username) {
        return null;
    }

    await fetch('/api/save-username', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ username }),
    });

    return username;
};
